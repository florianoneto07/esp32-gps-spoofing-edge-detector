#!/usr/bin/env bash
# Demo completa num terminal: Mosquitto + estação de solo + drone (ESP32 simulado).
#
# Uso:  ./scripts/run_demo.sh [opções do src/drone_sim.py]
# Ex.:  ./scripts/run_demo.sh --attack drift --attack-start 20 --duration 60
#
# - Se já existe um broker na porta 1883 (por exemplo, o serviço mosquitto do
#   sistema), ele é reaproveitado; senão a demo sobe um com
#   config/mosquitto.conf e o derruba no final.
# - A estação de solo roda em segundo plano e imprime neste terminal; o drone
#   roda em primeiro plano em modo --quiet (só eventos). Ctrl+C encerra tudo.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PY="$ROOT/.venv/bin/python"
PORT="${MQTT_PORT:-1883}"
if [ ! -x "$PY" ]; then
    echo "[demo] venv não encontrado em .venv/ (veja a seção Instalação do README)." >&2
    exit 1
fi
mkdir -p results

BROKER_PID=""
GS_PID=""

cleanup() {
    trap - EXIT INT TERM HUP
    if [ -n "$GS_PID" ] && kill -0 "$GS_PID" 2>/dev/null; then
        kill -TERM "$GS_PID" 2>/dev/null || true
        wait "$GS_PID" 2>/dev/null || true
    fi
    if [ -n "$BROKER_PID" ] && kill -0 "$BROKER_PID" 2>/dev/null; then
        echo "[demo] encerrando o mosquitto iniciado pela demo"
        kill -TERM "$BROKER_PID" 2>/dev/null || true
        wait "$BROKER_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM HUP

port_in_use() {
    ss -ltn 2>/dev/null | awk '{print $4}' | grep -Eq "[:.]${PORT}\$"
}

if port_in_use; then
    echo "[demo] já existe um broker na porta $PORT; reaproveitando."
else
    if ! command -v mosquitto >/dev/null 2>&1; then
        echo "[demo] mosquitto não instalado: sudo apt install -y mosquitto mosquitto-clients" >&2
        exit 1
    fi
    echo "[demo] subindo mosquitto com config/mosquitto.conf (log em results/mosquitto.log)"
    # setsid: o Ctrl+C do terminal não derruba o broker antes de o drone avisar "offline".
    setsid mosquitto -c config/mosquitto.conf -v >results/mosquitto.log 2>&1 &
    BROKER_PID=$!
    for _ in $(seq 1 40); do
        port_in_use && break
        sleep 0.25
    done
    if ! port_in_use; then
        echo "[demo] o mosquitto não subiu; veja results/mosquitto.log" >&2
        exit 1
    fi
fi

# MQTT_HOST/MQTT_PORT, se definidos, valem para os dois programas (src/config.py).
setsid "$PY" src/ground_station.py &
GS_PID=$!
sleep 1

"$PY" src/drone_sim.py --quiet "$@"
