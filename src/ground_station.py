"""Estação de solo: assina os tópicos do drone, imprime e grava a sessão em CSV.

Assina telemetry, alerts, status e groundtruth de lab/drone01/, imprime linhas
legíveis (alertas em destaque) e grava uma linha por telemetria em
results/session_<timestamp>.csv, já juntando o ground truth da mesma amostra
(campo seq) para permitir avaliar a sessão depois.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import paho.mqtt.client as mqtt

import config

RESULTS_DIR = Path(__file__).resolve().parents[1] / "results"

CSV_FIELDS = [
    "recv_time",
    "seq",
    "ts",
    "lat",
    "lon",
    "alt",
    "gps_speed",
    "imu_vn",
    "imu_ve",
    "battery",
    "flag_speed",
    "flag_accel",
    "flag_dr_divergence",
    "score",
    "alert",
    "det_us",
    "spoof_active",
    "attack_mode",
]


def log(message: str) -> None:
    print(time.strftime("%H:%M:%S") + " [solo] " + message, flush=True)


class Style:
    """Cores ANSI apenas quando a saída é um terminal."""

    def __init__(self, enabled: bool):
        self.enabled = enabled

    def _wrap(self, code: str, text: str) -> str:
        return "\033[%sm%s\033[0m" % (code, text) if self.enabled else text

    def alert(self, text: str) -> str:
        return self._wrap("1;97;41", text)

    def ok(self, text: str) -> str:
        return self._wrap("1;32", text)

    def warn(self, text: str) -> str:
        return self._wrap("1;33", text)


def _as_bit(value):
    if value is None:
        return ""
    return 1 if value else 0


class SessionRecorder:
    """Grava a sessão em CSV juntando telemetria e ground truth pelo seq."""

    def __init__(self, path: Path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(str(path), "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS)
        self._writer.writeheader()
        self._groundtruth = {}  # seq -> (spoof_active, mode)
        self.rows = 0

    def add_groundtruth(self, seq, active, mode) -> None:
        self._groundtruth[seq] = (active, mode)
        if len(self._groundtruth) > 300:  # guarda só as entradas recentes
            for old in sorted(self._groundtruth)[:-100]:
                del self._groundtruth[old]

    def add_telemetry(self, data: dict):
        """Grava uma linha; devolve o ground truth da amostra (ou None)."""
        gt = self._groundtruth.pop(data.get("seq"), None)
        flags = data.get("flags") or {}
        row = {
            "recv_time": "%.3f" % time.time(),
            "seq": data.get("seq"),
            "ts": data.get("ts"),
            "lat": data.get("lat"),
            "lon": data.get("lon"),
            "alt": data.get("alt"),
            "gps_speed": data.get("gps_speed"),
            "imu_vn": data.get("imu_vn"),
            "imu_ve": data.get("imu_ve"),
            "battery": data.get("battery"),
            "flag_speed": _as_bit(flags.get("speed")),
            "flag_accel": _as_bit(flags.get("accel")),
            "flag_dr_divergence": _as_bit(flags.get("dr_divergence")),
            "score": data.get("score"),
            "alert": _as_bit(data.get("alert")),
            "det_us": data.get("det_us"),
            "spoof_active": "" if gt is None else _as_bit(gt[0]),
            "attack_mode": "" if gt is None else gt[1],
        }
        self._writer.writerow(row)
        self._file.flush()  # o CSV fica utilizável mesmo se o processo morrer
        self.rows += 1
        return gt

    def close(self) -> None:
        self._file.close()


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


class GroundStation:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        use_color = sys.stdout.isatty() and not args.no_color and "NO_COLOR" not in os.environ
        self.style = Style(use_color)
        csv_path = Path(args.csv) if args.csv else RESULTS_DIR / ("session_%s.csv" % datetime.now().strftime("%Y%m%d_%H%M%S"))
        self.recorder = SessionRecorder(csv_path)
        self.alerts = 0
        self._last_gt_active = None

        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id="ground-station-%d" % os.getpid(),
            protocol=mqtt.MQTTv311,
        )
        self.client.reconnect_delay_set(config.RECONNECT_MIN_DELAY_S, config.RECONNECT_MAX_DELAY_S)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

    # --- callbacks -------------------------------------------------------------

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            log("broker recusou a conexão: %s" % reason_code)
            return
        log("conectado a %s:%d; assinando %s/{telemetry,alerts,status,groundtruth}" % (self.args.host, self.args.port, config.TOPIC_BASE))
        client.subscribe(
            [
                (config.TOPIC_TELEMETRY, config.QOS_TELEMETRY),
                (config.TOPIC_ALERTS, config.QOS_ALERTS),
                (config.TOPIC_STATUS, config.QOS_STATUS),
                (config.TOPIC_GROUNDTRUTH, config.QOS_GROUNDTRUTH),
            ]
        )

    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        log("desconectado do broker (%s)" % reason_code)

    def _on_message(self, client, userdata, msg):
        # Uma mensagem malformada não pode derrubar a estação de solo.
        try:
            if msg.topic == config.TOPIC_STATUS:
                self._handle_status(msg.payload.decode("utf-8", "replace"), msg.retain)
                return
            data = json.loads(msg.payload.decode("utf-8"))
            if not isinstance(data, dict):
                raise ValueError("payload não é um objeto JSON")
            if msg.topic == config.TOPIC_TELEMETRY:
                self._handle_telemetry(data)
            elif msg.topic == config.TOPIC_ALERTS:
                self._handle_alert(data)
            elif msg.topic == config.TOPIC_GROUNDTRUTH:
                self._handle_groundtruth(data)
        except Exception as exc:  # noqa: BLE001 - registrar e seguir
            log("mensagem inválida em %s: %s" % (msg.topic, exc))

    # --- tratamento por tópico -----------------------------------------------

    def _handle_status(self, status: str, retained: bool) -> None:
        text = "STATUS %s: %s%s" % (config.DRONE_ID, status, " (retida)" if retained else "")
        log(self.style.ok(text) if status == config.STATUS_ONLINE else self.style.warn(text))

    def _handle_groundtruth(self, data: dict) -> None:
        active = bool(data.get("spoof_active"))
        mode = data.get("mode", "")
        self.recorder.add_groundtruth(data.get("seq"), active, mode)
        if active != self._last_gt_active:
            if active:
                log(self.style.warn("GROUND TRUTH: spoofing ATIVO (modo %s) a partir do seq %s" % (mode, data.get("seq"))))
            elif self._last_gt_active is not None:
                log(self.style.ok("GROUND TRUTH: spoofing encerrado no seq %s" % data.get("seq")))
            self._last_gt_active = active

    def _handle_telemetry(self, data: dict) -> None:
        gt = self.recorder.add_telemetry(data)
        if self.args.quiet:
            return
        flags = data.get("flags") or {}
        flag_txt = "".join(
            letter if flags.get(name) else "-" for letter, name in (("S", "speed"), ("A", "accel"), ("D", "dr_divergence"))
        )
        gt_txt = "" if gt is None else ("  GT:ataque" if gt[0] else "  GT:normal")
        line = "TEL seq=%4s lat=%.6f lon=%.6f alt=%5.1f v=%4.1f m/s bat=%5.1f%% score=%5.2f flags=%s%s" % (
            data.get("seq"),
            data.get("lat", float("nan")),
            data.get("lon", float("nan")),
            data.get("alt", float("nan")),
            data.get("gps_speed", float("nan")),
            data.get("battery", float("nan")),
            data.get("score", float("nan")),
            flag_txt,
            gt_txt,
        )
        if data.get("alert"):
            line += "  " + self.style.alert(" ALERTA ATIVO ")
        log(line)

    def _handle_alert(self, data: dict) -> None:
        self.alerts += 1
        d = data.get("details") or {}
        nan = float("nan")
        log(
            self.style.alert(" !! ALERTA DE GPS SPOOFING !! ")
            + " seq=%s tipo=%s score=%.2f | v_impl=%.1f m/s a_impl=%.1f m/s² div_DR=%.1f m"
            % (
                data.get("seq"),
                data.get("type"),
                data.get("score", nan),
                d.get("implied_speed_mps", nan),
                d.get("implied_accel_mps2", nan),
                d.get("dr_divergence_m", nan),
            )
        )

    # --- execução ----------------------------------------------------------------

    def run(self) -> None:
        signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
        log("broker %s:%d | gravando em %s | Ctrl+C encerra" % (self.args.host, self.args.port, self.recorder.path))
        self.client.connect_async(self.args.host, self.args.port, keepalive=config.KEEPALIVE_S)
        try:
            # Tenta conectar até o broker aparecer e reconecta se ele cair.
            self.client.loop_forever(retry_first_connection=True)
        except KeyboardInterrupt:
            pass
        finally:
            self.client.disconnect()
            self.recorder.close()
            log("sessão encerrada: %d telemetrias e %d alertas gravados em %s" % (self.recorder.rows, self.alerts, self.recorder.path))


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Estação de solo: assina os tópicos do drone e grava CSV.")
    parser.add_argument("--host", default=config.BROKER_HOST, help="broker MQTT (padrão: %(default)s)")
    parser.add_argument("--port", type=int, default=config.BROKER_PORT, help="porta do broker (padrão: %(default)s)")
    parser.add_argument("--csv", default=None, help="caminho do CSV (padrão: results/session_<timestamp>.csv)")
    parser.add_argument("--quiet", action="store_true", help="não imprime cada telemetria (só alertas, status e ground truth)")
    parser.add_argument("--no-color", action="store_true", help="desativa as cores ANSI")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    GroundStation(parse_args(argv)).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
