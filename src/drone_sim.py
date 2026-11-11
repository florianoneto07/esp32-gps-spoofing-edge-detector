"""Firmware simulado do ESP32 embarcado no drone.

A cada amostra (1 Hz):
  1. avança o voo real e lê os sensores simulados (GPS e IMU);
  2. roda o detector de borda ANTES de publicar e mede o tempo gasto (µs);
  3. publica telemetria (QoS 0) e ground truth (QoS 0) e, quando o detector
     dispara, um alerta (QoS 1).

O spoofing afeta só o GPS (a IMU não é afetada) e pode ser acionado pela
linha de comando (--attack/--attack-start) ou pelo tópico lab/drone01/cmd/spoof,
por exemplo a partir do MQTT X.

As classes de simulação (DroneSimulator e auxiliares) não dependem de MQTT:
os testes e o scripts/evaluate.py as importam e rodam tudo offline.
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import queue
import random
import signal
import statistics
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import config
from edge_detector import EdgeSpoofDetector
from geo import offset_latlon

try:
    import paho.mqtt.client as mqtt
except ImportError:  # o simulador offline funciona mesmo sem o paho
    mqtt = None


# ---------------------------------------------------------------------------
# Modelos da simulação (sem MQTT)
# ---------------------------------------------------------------------------


class GaussMarkov:
    """Ruído correlacionado no tempo: processo de Gauss-Markov de 1a ordem."""

    def __init__(self, sigma: float, tau_s: float, dt: float, rng: random.Random):
        self._rng = rng
        self._phi = math.exp(-dt / tau_s)
        self._q = sigma * math.sqrt(1.0 - self._phi ** 2)
        self.value = rng.gauss(0.0, sigma)  # já começa no regime estacionário

    def step(self) -> float:
        self.value = self._phi * self.value + self._rng.gauss(0.0, self._q)
        return self.value


class FlightModel:
    """Voo real: segue os waypoints em loop, em velocidade de cruzeiro e com
    aceleração limitada. Coordenadas locais (leste, norte) em metros."""

    def __init__(
        self,
        waypoints=config.WAYPOINTS_EN_M,
        speed: float = config.CRUISE_SPEED_MPS,
        max_accel: float = config.MAX_ACCEL_MPS2,
        radius: float = config.WAYPOINT_RADIUS_M,
        substeps: int = config.PHYSICS_SUBSTEPS,
    ):
        if len(waypoints) < 2:
            raise ValueError("a rota precisa de pelo menos 2 waypoints")
        self._waypoints = [(float(e), float(n)) for e, n in waypoints]
        self._speed = speed
        self._max_accel = max_accel
        self._radius = radius
        self._substeps = substeps
        self.east, self.north = self._waypoints[0]
        self._target = 1
        # Começa já em cruzeiro, apontando para o próximo waypoint.
        self.ve, self.vn = self._desired_velocity()

    def _vector_to_target(self) -> Tuple[float, float, float]:
        target_e, target_n = self._waypoints[self._target]
        dx = target_e - self.east
        dy = target_n - self.north
        return dx, dy, math.hypot(dx, dy)

    def _desired_velocity(self) -> Tuple[float, float]:
        dx, dy, dist = self._vector_to_target()
        if dist < self._radius:  # waypoint atingido: segue para o próximo
            self._target = (self._target + 1) % len(self._waypoints)
            dx, dy, dist = self._vector_to_target()
        if dist < 1e-9:
            return 0.0, 0.0
        return self._speed * dx / dist, self._speed * dy / dist

    def step(self, dt: float) -> None:
        h = dt / self._substeps
        max_dv = self._max_accel * h
        for _ in range(self._substeps):
            want_e, want_n = self._desired_velocity()
            dve = want_e - self.ve
            dvn = want_n - self.vn
            mag = math.hypot(dve, dvn)
            if mag > max_dv:  # limita a aceleração (curvas e frenagens suaves)
                dve *= max_dv / mag
                dvn *= max_dv / mag
            self.ve += dve
            self.vn += dvn
            self.east += self.ve * h
            self.north += self.vn * h


class ImuModel:
    """Velocidade inercial = velocidade real + viés (opcional) + ruído branco."""

    def __init__(self, sigma: float, bias_sigma: float, rng: random.Random):
        self._rng = rng
        self._sigma = sigma
        # O viés é sorteado uma vez por execução; com bias_sigma = 0 ele é nulo,
        # mas os sorteios acontecem do mesmo jeito para manter a sequência de
        # ruído idêntica entre execuções com a mesma seed.
        self.bias_e = rng.gauss(0.0, bias_sigma)
        self.bias_n = rng.gauss(0.0, bias_sigma)

    def measure(self, ve: float, vn: float) -> Tuple[float, float]:
        return (
            ve + self.bias_e + self._rng.gauss(0.0, self._sigma),
            vn + self.bias_n + self._rng.gauss(0.0, self._sigma),
        )


class GpsModel:
    """Receptor GPS: posição real + erro correlacionado + ruído branco + spoofing."""

    def __init__(self, rng: random.Random, dt: float = config.SAMPLE_PERIOD_S):
        self._rng = rng
        self._err_e = GaussMarkov(config.GPS_SIGMA_M, config.GPS_CORRELATION_S, dt, rng)
        self._err_n = GaussMarkov(config.GPS_SIGMA_M, config.GPS_CORRELATION_S, dt, rng)
        self._err_alt = GaussMarkov(config.GPS_ALT_SIGMA_M, config.GPS_CORRELATION_S, dt, rng)

    def measure(self, east, north, alt, ve, vn, spoof_offset, spoof_velocity):
        white = config.GPS_WHITE_SIGMA_M
        meas_e = east + self._err_e.step() + self._rng.gauss(0.0, white) + spoof_offset[0]
        meas_n = north + self._err_n.step() + self._rng.gauss(0.0, white) + spoof_offset[1]
        lat, lon = offset_latlon(config.HOME_LAT, config.HOME_LON, meas_n, meas_e)
        meas_alt = alt + self._err_alt.step()
        # Velocidade Doppler: um spoofer consistente faz o receptor "ver" a
        # velocidade da trajetória falsa (real + deriva).
        speed = math.hypot(ve + spoof_velocity[0], vn + spoof_velocity[1])
        speed += self._rng.gauss(0.0, config.GPS_SPEED_SIGMA_MPS)
        return lat, lon, meas_alt, max(0.0, speed)


class SpoofAttack:
    """Estado do ataque de spoofing. Afeta apenas o GPS.

    - jump: desloca a posição reportada em 150-300 m de uma vez (direção
      aleatória) e mantém esse offset;
    - drift: o offset cresce a drift_mps numa direção fixa (aleatória).
    """

    def __init__(self, rng: random.Random):
        self._rng = rng
        self.mode = "none"
        self.drift_mps = 0.0
        self.offset_e = 0.0
        self.offset_n = 0.0
        self._dir_e = 0.0
        self._dir_n = 0.0

    @property
    def active(self) -> bool:
        return self.mode != "none"

    @property
    def offset(self) -> Tuple[float, float]:
        return self.offset_e, self.offset_n

    @property
    def velocity(self) -> Tuple[float, float]:
        if self.mode != "drift":
            return 0.0, 0.0
        return self._dir_e * self.drift_mps, self._dir_n * self.drift_mps

    def set_mode(self, mode: str, drift_mps: Optional[float] = None) -> None:
        if mode not in config.ATTACK_MODES:
            raise ValueError("modo de ataque inválido: %r" % mode)
        rate = config.DRIFT_DEFAULT_MPS if drift_mps is None else float(drift_mps)
        if mode == "drift" and self.mode == "drift":
            # Ajuste de taxa com a deriva em curso: mantém offset e direção.
            self.drift_mps = rate
            return
        self.mode = mode
        self.offset_e = self.offset_n = 0.0
        self.drift_mps = 0.0
        if mode == "jump":
            dist = self._rng.uniform(config.JUMP_MIN_M, config.JUMP_MAX_M)
            angle = self._rng.uniform(0.0, 2.0 * math.pi)
            self.offset_e = dist * math.sin(angle)
            self.offset_n = dist * math.cos(angle)
        elif mode == "drift":
            angle = self._rng.uniform(0.0, 2.0 * math.pi)
            self._dir_e = math.sin(angle)
            self._dir_n = math.cos(angle)
            self.drift_mps = rate

    def step(self, dt: float) -> None:
        if self.mode == "drift":
            self.offset_e += self._dir_e * self.drift_mps * dt
            self.offset_n += self._dir_n * self.drift_mps * dt


@dataclass
class SensorSample:
    """Uma amostra do drone: o que o ESP32 lê + ground truth para avaliação."""

    seq: int
    t: float  # tempo de simulação (s)
    lat: float  # GPS (possivelmente falsificado)
    lon: float
    alt: float
    gps_speed: float
    imu_vn: float
    imu_ve: float
    battery: float
    spoof_active: bool  # ground truth
    attack_mode: str
    true_lat: float
    true_lon: float


class DroneSimulator:
    """Núcleo da simulação, sem MQTT: voo real, sensores e ataque."""

    def __init__(
        self,
        seed: Optional[int] = None,
        imu_bias_sigma: float = config.IMU_BIAS_SIGMA_MPS,
        dt: float = config.SAMPLE_PERIOD_S,
    ):
        self.dt = dt
        self._rng = random.Random(seed)
        # RNG separado para o ataque: a mesma seed gera o mesmo ruído de
        # sensor com ou sem ataque, o que permite comparações pareadas.
        self._attack_rng = random.Random(None if seed is None else "attack-%d" % seed)
        self.flight = FlightModel()
        self.gps = GpsModel(self._rng, dt)
        self.imu = ImuModel(config.IMU_VEL_SIGMA_MPS, imu_bias_sigma, self._rng)
        self.attack = SpoofAttack(self._attack_rng)
        self.seq = 0

    def set_attack(self, mode: str, drift_mps: Optional[float] = None) -> None:
        self.attack.set_mode(mode, drift_mps)

    def step(self) -> SensorSample:
        """Gera a próxima amostra (a primeira corresponde a t = 0)."""
        if self.seq > 0:
            self.flight.step(self.dt)
        t = self.seq * self.dt
        if self.attack.active:
            self.attack.step(self.dt)
        f = self.flight
        lat, lon, alt, gps_speed = self.gps.measure(
            f.east, f.north, config.CRUISE_ALT_M, f.ve, f.vn, self.attack.offset, self.attack.velocity
        )
        imu_ve, imu_vn = self.imu.measure(f.ve, f.vn)
        true_lat, true_lon = offset_latlon(config.HOME_LAT, config.HOME_LON, f.north, f.east)
        battery = max(config.BATTERY_MIN_PCT, 100.0 - config.BATTERY_DRAIN_PCT_PER_S * t)
        sample = SensorSample(
            seq=self.seq,
            t=t,
            lat=lat,
            lon=lon,
            alt=alt,
            gps_speed=gps_speed,
            imu_vn=imu_vn,
            imu_ve=imu_ve,
            battery=battery,
            spoof_active=self.attack.active,
            attack_mode=self.attack.mode,
            true_lat=true_lat,
            true_lon=true_lon,
        )
        self.seq += 1
        return sample


def parse_spoof_command(payload: bytes) -> Tuple[str, Optional[float]]:
    """Valida o JSON recebido em cmd/spoof, ex.: {"mode":"drift","drift_mps":2.0}."""
    try:
        data = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as exc:
        raise ValueError("payload não é JSON válido (%s)" % exc)
    if not isinstance(data, dict):
        raise ValueError("payload deve ser um objeto JSON")
    mode = str(data.get("mode", "")).strip().lower()
    if mode not in config.ATTACK_MODES:
        raise ValueError("mode inválido: %r (use none, jump ou drift)" % data.get("mode"))
    drift = None
    if mode == "drift":
        try:
            drift = float(data.get("drift_mps", config.DRIFT_DEFAULT_MPS))
        except (TypeError, ValueError):
            raise ValueError("drift_mps deve ser numérico")
        if not 0.0 < drift <= config.DRIFT_MAX_MPS:
            raise ValueError("drift_mps fora do intervalo (0, %g]" % config.DRIFT_MAX_MPS)
    return mode, drift


# ---------------------------------------------------------------------------
# "Firmware": laço de 1 Hz com MQTT
# ---------------------------------------------------------------------------


def log(message: str) -> None:
    print(time.strftime("%H:%M:%S") + " [drone] " + message, flush=True)


def _raise_keyboard_interrupt(signum, frame):
    # SIGTERM (kill, run_demo.sh) passa pelo mesmo encerramento limpo do Ctrl+C.
    raise KeyboardInterrupt


class DroneFirmware:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.sim = DroneSimulator(seed=args.seed, imu_bias_sigma=args.imu_bias)
        self.detector = EdgeSpoofDetector()
        self._commands: "queue.Queue[Tuple[str, Optional[float]]]" = queue.Queue()
        self._stop = threading.Event()
        self._connected = threading.Event()
        self._det_us = collections.deque(maxlen=86400)  # tempo do detector por amostra
        self._alerts_sent = 0

        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=config.DRONE_ID + "-esp32-sim",
            protocol=mqtt.MQTTv311,
        )
        # Last Will: se a conexão cair sem DISCONNECT, o broker publica "offline".
        self.client.will_set(config.TOPIC_STATUS, config.STATUS_OFFLINE, qos=config.QOS_STATUS, retain=True)
        self.client.reconnect_delay_set(config.RECONNECT_MIN_DELAY_S, config.RECONNECT_MAX_DELAY_S)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

    # --- callbacks (executam na thread de rede do paho) ---------------------

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            log("broker recusou a conexão: %s" % reason_code)
            return
        log("conectado a %s:%d" % (self.args.host, self.args.port))
        client.publish(config.TOPIC_STATUS, config.STATUS_ONLINE, qos=config.QOS_STATUS, retain=True)
        client.subscribe(config.TOPIC_CMD_SPOOF, qos=config.QOS_CMD)
        self._connected.set()

    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        self._connected.clear()
        if not self._stop.is_set():
            log("desconectado do broker (%s); reconectando automaticamente..." % reason_code)

    def _on_message(self, client, userdata, msg):
        try:
            self._commands.put(parse_spoof_command(msg.payload))
        except ValueError as exc:
            log("comando ignorado em %s: %s" % (msg.topic, exc))

    # --- laço principal ------------------------------------------------------

    def _set_attack(self, mode: str, drift_mps: Optional[float], origin: str) -> None:
        self.sim.set_attack(mode, drift_mps)
        detail = " (%.1f m/s)" % self.sim.attack.drift_mps if mode == "drift" else ""
        log("ataque no GPS -> %s%s [origem: %s]" % (mode, detail, origin))

    def _apply_commands(self) -> None:
        while True:
            try:
                mode, drift = self._commands.get_nowait()
            except queue.Empty:
                return
            self._set_attack(mode, drift, "MQTT " + config.TOPIC_CMD_SPOOF)

    def run(self) -> None:
        args = self.args
        signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
        log("iniciando %s | broker %s:%d | Ctrl+C encerra" % (config.DRONE_ID, args.host, args.port))
        self.client.connect_async(args.host, args.port, keepalive=config.KEEPALIVE_S)
        self.client.loop_start()  # thread de rede: reconecta sozinha se o broker cair

        attack_pending = args.attack != "none"
        prev_alert = False
        try:
            # Espera a primeira conexão para não perder as primeiras telemetrias
            # (QoS 0 sem conexão é descartada). Sem broker, segue mesmo assim.
            if not self._connected.wait(timeout=5.0):
                log("broker indisponível; amostrando assim mesmo e tentando reconectar em segundo plano")
            start_wall = time.time()
            next_tick = time.monotonic()
            while not self._stop.is_set():
                self._apply_commands()
                if attack_pending and self.sim.seq >= args.attack_start:
                    self._set_attack(args.attack, args.drift_mps, "linha de comando")
                    attack_pending = False

                sample = self.sim.step()
                ts = start_wall + sample.t

                # Detector roda ANTES da publicação; o tempo dele é medido.
                t0 = time.perf_counter_ns()
                result = self.detector.update(ts, sample.lat, sample.lon, sample.imu_vn, sample.imu_ve)
                det_us = (time.perf_counter_ns() - t0) / 1000.0
                self._det_us.append(det_us)

                self._publish(sample, ts, result, det_us, new_alert=result.alert and not prev_alert)
                prev_alert = result.alert

                if args.duration and self.sim.seq >= args.duration:
                    break
                next_tick += self.sim.dt
                self._stop.wait(max(0.0, next_tick - time.monotonic()))
        except KeyboardInterrupt:
            pass
        finally:
            self._shutdown()

    def _publish(self, s: SensorSample, ts: float, r, det_us: float, new_alert: bool) -> None:
        groundtruth = {"seq": s.seq, "spoof_active": s.spoof_active, "mode": s.attack_mode}
        self.client.publish(config.TOPIC_GROUNDTRUTH, json.dumps(groundtruth), qos=config.QOS_GROUNDTRUTH)

        telemetry = {
            "id": config.DRONE_ID,
            "seq": s.seq,
            "ts": round(ts, 3),
            "lat": round(s.lat, 7),
            "lon": round(s.lon, 7),
            "alt": round(s.alt, 1),
            "gps_speed": round(s.gps_speed, 2),
            "imu_vn": round(s.imu_vn, 2),
            "imu_ve": round(s.imu_ve, 2),
            "battery": round(s.battery, 1),
            "flags": r.flags,
            "score": round(r.score, 3),
            "alert": r.alert,
            "det_us": round(det_us, 1),
        }
        self.client.publish(config.TOPIC_TELEMETRY, json.dumps(telemetry), qos=config.QOS_TELEMETRY)

        if new_alert:
            alert = {
                "seq": s.seq,
                "ts": round(ts, 3),
                "type": r.alert_type,
                "score": round(r.score, 3),
                "details": {key: round(value, 2) for key, value in r.details.items()},
            }
            self.client.publish(config.TOPIC_ALERTS, json.dumps(alert), qos=config.QOS_ALERTS)
            self._alerts_sent += 1
            log("ALERTA publicado em %s: tipo=%s score=%.2f seq=%d" % (config.TOPIC_ALERTS, r.alert_type, r.score, s.seq))

        if not self.args.quiet:
            log(self._format_line(s, r, det_us))

    def _format_line(self, s: SensorSample, r, det_us: float) -> str:
        flags = "".join(
            letter if r.flags.get(name) else "-"
            for letter, name in (("S", "speed"), ("A", "accel"), ("D", "dr_divergence"))
        )
        if not s.spoof_active:
            attack = "sem ataque"
        elif s.attack_mode == "drift":
            attack = "ataque drift %.1f m/s" % self.sim.attack.drift_mps
        else:
            attack = "ataque " + s.attack_mode
        line = "seq=%4d lat=%.6f lon=%.6f v=%4.1f m/s div=%6.1f m score=%5.2f flags=%s det=%5.1f µs [%s]" % (
            s.seq,
            s.lat,
            s.lon,
            s.gps_speed,
            r.details.get("dr_divergence_m", float("nan")),
            r.score,
            flags,
            det_us,
            attack,
        )
        return line + ("  << ALERTA" if r.alert else "")

    def _shutdown(self) -> None:
        self._stop.set()
        try:
            info = self.client.publish(config.TOPIC_STATUS, config.STATUS_OFFLINE, qos=config.QOS_STATUS, retain=True)
            if self.client.is_connected():
                info.wait_for_publish(timeout=2.0)
        except (RuntimeError, ValueError):
            pass
        self.client.disconnect()
        self.client.loop_stop()
        log("encerrado: %d amostras, %d alertas publicados; status 'offline' enviado" % (self.sim.seq, self._alerts_sent))
        self._log_timing()

    def _log_timing(self) -> None:
        if not self._det_us:
            return
        data = sorted(self._det_us)
        n = len(data)

        def pct(q):
            return data[min(n - 1, int(round(q * (n - 1))))]

        log(
            "tempo do detector (%d amostras): média %.1f µs | p50 %.1f | p99 %.1f | máx %.1f µs "
            "(CPython nesta máquina; ver README sobre o ESP32)" % (n, statistics.mean(data), pct(0.5), pct(0.99), data[-1])
        )


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ESP32 simulado: detecção de GPS spoofing na borda + MQTT.")
    parser.add_argument("--host", default=config.BROKER_HOST, help="broker MQTT (padrão: %(default)s)")
    parser.add_argument("--port", type=int, default=config.BROKER_PORT, help="porta do broker (padrão: %(default)s)")
    parser.add_argument("--attack", choices=config.ATTACK_MODES, default="none", help="ataque agendado")
    parser.add_argument("--attack-start", type=int, default=60, metavar="N", help="segundos até o ataque agendado (padrão: %(default)s)")
    parser.add_argument("--drift-mps", type=float, default=config.DRIFT_DEFAULT_MPS, help="taxa da deriva em m/s (padrão: %(default)s)")
    parser.add_argument("--duration", type=int, default=0, help="duração em segundos; 0 = até Ctrl+C")
    parser.add_argument("--seed", type=int, default=None, help="seed do ruído (padrão: aleatória)")
    parser.add_argument("--imu-bias", type=float, default=config.IMU_BIAS_SIGMA_MPS, help="desvio do viés da IMU, m/s (padrão: %(default)s)")
    parser.add_argument("--quiet", action="store_true", help="imprime só eventos (ataques, alertas, conexão)")
    args = parser.parse_args(argv)
    if args.attack == "drift" and not 0.0 < args.drift_mps <= config.DRIFT_MAX_MPS:
        parser.error("--drift-mps deve estar em (0, %g]" % config.DRIFT_MAX_MPS)
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    if mqtt is None:
        print("paho-mqtt não encontrado: ative o venv e rode 'pip install -r requirements.txt'")
        return 1
    DroneFirmware(args).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
