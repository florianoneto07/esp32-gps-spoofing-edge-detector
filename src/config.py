"""Configuração central do projeto.

Todas as constantes ficam aqui (broker, tópicos, parâmetros da simulação e
limiares do detector) para que o firmware simulado, a estação de solo, os
testes e a avaliação usem exatamente os mesmos valores.
"""

import os

# ---------------------------------------------------------------------------
# Identificação
# ---------------------------------------------------------------------------
DRONE_ID = "drone01"

# ---------------------------------------------------------------------------
# Broker MQTT (laboratório: sem TLS e sem autenticação, ver README)
# ---------------------------------------------------------------------------
BROKER_HOST = os.environ.get("MQTT_HOST", "localhost")
BROKER_PORT = int(os.environ.get("MQTT_PORT", "1883"))
KEEPALIVE_S = 30
RECONNECT_MIN_DELAY_S = 1
RECONNECT_MAX_DELAY_S = 30

TOPIC_BASE = "lab/" + DRONE_ID
TOPIC_TELEMETRY = TOPIC_BASE + "/telemetry"
TOPIC_ALERTS = TOPIC_BASE + "/alerts"
TOPIC_STATUS = TOPIC_BASE + "/status"
TOPIC_CMD_SPOOF = TOPIC_BASE + "/cmd/spoof"
TOPIC_GROUNDTRUTH = TOPIC_BASE + "/groundtruth"

QOS_TELEMETRY = 0
QOS_ALERTS = 1
QOS_STATUS = 1
QOS_CMD = 1
QOS_GROUNDTRUTH = 0

STATUS_ONLINE = "online"
STATUS_OFFLINE = "offline"

# ---------------------------------------------------------------------------
# Voo simulado
# ---------------------------------------------------------------------------
HOME_LAT = -23.5505  # coordenada de referência (centro de São Paulo)
HOME_LON = -46.6333
CRUISE_SPEED_MPS = 8.0
CRUISE_ALT_M = 50.0
SAMPLE_PERIOD_S = 1.0  # amostragem do detector e publicação: 1 Hz
PHYSICS_SUBSTEPS = 10  # a posição real é integrada a 10 Hz internamente
MAX_ACCEL_MPS2 = 2.0  # aceleração máxima do drone (curvas suaves)
WAYPOINT_RADIUS_M = 12.0  # distância para considerar o waypoint atingido

# Rota em metros (leste, norte) relativos a HOME, percorrida em loop.
WAYPOINTS_EN_M = (
    (0.0, 0.0),
    (400.0, 0.0),
    (400.0, 300.0),
    (100.0, 450.0),
    (-200.0, 250.0),
)

BATTERY_DRAIN_PCT_PER_S = 0.02
BATTERY_MIN_PCT = 5.0

# ---------------------------------------------------------------------------
# Modelos de sensor
# ---------------------------------------------------------------------------
# GPS: erro de posição com desvio-padrão de 2 m por eixo. Como nos receptores
# reais, esse erro é correlacionado no tempo (processo de Gauss-Markov de 1a
# ordem) e há uma componente branca pequena entre fixes consecutivos. Com
# ruído branco puro de 2 m a 1 Hz, a aceleração implícita teria desvio de
# ~4,9 m/s² por eixo e a regra de 8 m/s² dispararia em ~26% das amostras.
GPS_SIGMA_M = 2.0
GPS_CORRELATION_S = 60.0
GPS_WHITE_SIGMA_M = 0.5
GPS_ALT_SIGMA_M = 3.0
GPS_SPEED_SIGMA_MPS = 0.1

# Velocidade inercial ("IMU"): velocidade real + ruído branco por eixo.
# Premissa: fonte de velocidade independente do GPS (IMU auxiliada por fluxo
# óptico, por exemplo). O viés opcional é sorteado por execução e por eixo e
# é usado na análise de sensibilidade; 0 mantém o modelo padrão.
IMU_VEL_SIGMA_MPS = 0.3
IMU_BIAS_SIGMA_MPS = 0.0

# ---------------------------------------------------------------------------
# Ataques de spoofing (aplicados somente ao GPS)
# ---------------------------------------------------------------------------
ATTACK_MODES = ("none", "jump", "drift")
JUMP_MIN_M = 150.0
JUMP_MAX_M = 300.0
DRIFT_DEFAULT_MPS = 2.0
DRIFT_MAX_MPS = 50.0

# ---------------------------------------------------------------------------
# Detector de borda
# ---------------------------------------------------------------------------
V_MAX_MPS = 25.0  # velocidade implícita máxima plausível
A_MAX_MPS2 = 8.0  # aceleração implícita máxima plausível
D_MAX_M = 25.0  # divergência máxima entre GPS e dead reckoning
RESYNC_S = 30.0  # período de re-ancoragem do dead reckoning no GPS
K_CONSECUTIVE = 2  # amostras suspeitas consecutivas para emitir alerta
