"""Detector de GPS spoofing executado na borda (no próprio ESP32).

Depende só de `math`, de `geo` e das constantes de `config`: nada de MQTT,
arquivos ou estruturas que cresçam com o tempo, para que o mesmo algoritmo
possa ser portado para C ou MicroPython. O estado guardado entre amostras é
mínimo: último fix GPS, último vetor velocidade implícito, última velocidade
da IMU, posição de dead reckoning, instante da última re-ancoragem e o
contador de amostras suspeitas.

Checagens, cada uma com sua flag:
  speed          velocidade implícita entre fixes GPS consecutivos > V_MAX
  accel          aceleração implícita (variação do vetor velocidade) > A_MAX
  dr_divergence  a posição de dead reckoning (integrada da velocidade da IMU)
                 diverge da posição GPS > D_MAX. O dead reckoning é
                 re-ancorado no GPS a cada RESYNC_S segundos, mas só quando a
                 amostra atual não é suspeita: durante um ataque a âncora fica
                 congelada e a divergência continua crescendo.

Uma amostra é suspeita se qualquer flag estiver ativa. O alerta só é emitido
após K amostras suspeitas consecutivas, o que filtra picos isolados de ruído.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional

import config
from geo import bearing_deg, haversine_m, offset_latlon

ALL_CHECKS = ("speed", "accel", "dr_divergence")


@dataclass
class DetectionResult:
    """Saída do detector para uma amostra."""

    flags: Dict[str, bool]  # flag de cada checagem habilitada
    score: float  # maior razão métrica/limiar; >= 1 indica checagem estourada
    alert: bool  # True após K amostras suspeitas consecutivas
    alert_type: Optional[str]  # checagens ativas no alerta, ex.: "accel+dr_divergence"
    details: Dict[str, float] = field(default_factory=dict)  # métricas da amostra


class EdgeSpoofDetector:
    """Detector com estado mínimo. Chame `update()` a cada fix GPS."""

    def __init__(
        self,
        checks: Iterable[str] = ALL_CHECKS,
        v_max: float = config.V_MAX_MPS,
        a_max: float = config.A_MAX_MPS2,
        d_max: float = config.D_MAX_M,
        resync_s: float = config.RESYNC_S,
        k_consecutive: int = config.K_CONSECUTIVE,
        resync_phase_s: float = 0.0,
    ):
        checks = tuple(checks)
        unknown = set(checks) - set(ALL_CHECKS)
        if unknown:
            raise ValueError("checagens desconhecidas: %s" % sorted(unknown))
        if k_consecutive < 1:
            raise ValueError("k_consecutive deve ser >= 1")
        # Mantém a ordem canônica nas flags e no alert_type.
        self.checks = tuple(name for name in ALL_CHECKS if name in checks)
        self.v_max = v_max
        self.a_max = a_max
        self.d_max = d_max
        self.resync_s = resync_s
        self.k_consecutive = k_consecutive
        # Fase inicial da janela de re-ancoragem. Num voo real o início de um
        # ataque cai em qualquer ponto da janela; a avaliação sorteia essa
        # fase para não favorecer nenhum caso. No firmware fica em 0.
        self.resync_phase_s = resync_phase_s
        self.reset()

    def reset(self) -> None:
        """Descarta todo o estado (equivale a reiniciar o dispositivo)."""
        self._last_fix = None  # (lat, lon, ts) do fix anterior
        self._last_vel = None  # (leste, norte) implícito do GPS, m/s
        self._last_imu = None  # (leste, norte) da IMU na amostra anterior, m/s
        self._dr = None  # (lat, lon) do dead reckoning
        self._anchor_ts = 0.0  # instante da última re-ancoragem
        self._consecutive = 0
        self._alert = False

    @property
    def alert_active(self) -> bool:
        return self._alert

    def update(self, ts: float, lat: float, lon: float, imu_vn: float, imu_ve: float) -> DetectionResult:
        """Processa um fix GPS (graus) e a velocidade da IMU (m/s, norte/leste)."""
        flags = {name: False for name in self.checks}
        details: Dict[str, float] = {}

        if self._last_fix is None:
            # Primeira amostra: só inicializa o estado e ancora o dead reckoning.
            self._last_fix = (lat, lon, ts)
            self._last_imu = (imu_ve, imu_vn)
            self._dr = (lat, lon)
            self._anchor_ts = ts - self.resync_phase_s
            return DetectionResult(flags, 0.0, False, None, details)

        prev_lat, prev_lon, prev_ts = self._last_fix
        dt = ts - prev_ts
        if dt <= 0.0:
            # Fix repetido ou fora de ordem: ignora sem mexer no estado.
            return DetectionResult(flags, 0.0, self._alert, None, details)

        ratios = []

        # 1) Velocidade implícita entre fixes consecutivos.
        dist = haversine_m(prev_lat, prev_lon, lat, lon)
        speed = dist / dt
        details["implied_speed_mps"] = speed
        if "speed" in flags:
            flags["speed"] = speed > self.v_max
            ratios.append(speed / self.v_max)

        # 2) Aceleração implícita: variação do vetor velocidade (pega também
        #    mudanças bruscas de direção, não só de módulo).
        if "accel" in flags:
            course = math.radians(bearing_deg(prev_lat, prev_lon, lat, lon))
            vel = (speed * math.sin(course), speed * math.cos(course))
            if self._last_vel is not None:
                accel = math.hypot(vel[0] - self._last_vel[0], vel[1] - self._last_vel[1]) / dt
                details["implied_accel_mps2"] = accel
                flags["accel"] = accel > self.a_max
                ratios.append(accel / self.a_max)
            self._last_vel = vel

        # 3) Dead reckoning: integra a velocidade da IMU (regra do trapézio) e
        #    compara com a posição GPS.
        if "dr_divergence" in flags:
            mean_ve = 0.5 * (self._last_imu[0] + imu_ve)
            mean_vn = 0.5 * (self._last_imu[1] + imu_vn)
            self._dr = offset_latlon(self._dr[0], self._dr[1], mean_vn * dt, mean_ve * dt)
            divergence = haversine_m(self._dr[0], self._dr[1], lat, lon)
            details["dr_divergence_m"] = divergence
            flags["dr_divergence"] = divergence > self.d_max
            ratios.append(divergence / self.d_max)

        suspicious = any(flags.values())
        self._consecutive = self._consecutive + 1 if suspicious else 0
        self._alert = self._consecutive >= self.k_consecutive

        # Re-ancoragem periódica, só com amostra limpa (sem suspeita nem alerta).
        if "dr_divergence" in flags and not suspicious and ts - self._anchor_ts >= self.resync_s:
            self._dr = (lat, lon)
            self._anchor_ts = ts

        self._last_fix = (lat, lon, ts)
        self._last_imu = (imu_ve, imu_vn)

        details["consecutive"] = self._consecutive
        alert_type = "+".join(name for name in self.checks if flags[name]) if self._alert else None
        score = max(ratios) if ratios else 0.0
        return DetectionResult(flags, score, self._alert, alert_type, details)
