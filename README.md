# esp32-gps-spoofing-edge-detector

Detecção de *GPS spoofing* **na borda**, ou seja, no próprio ESP32 embarcado num
drone, com telemetria e alertas publicados via MQTT (paho-mqtt + Mosquitto) e
visualização no MQTT X.

Este repositório é um MVP em simulação. O firmware do ESP32, o voo, os sensores e
os ataques são simulados em Python; o detector usa só aritmética simples e estado
mínimo para poder ser portado para o microcontrolador.

**Autor:** Floriano Neto · **Licença:** MIT

---

## Motivação e lacuna

Drones pequenos (sUAS) dependem do GPS civil, que não é autenticado. Um spoofer
pode empurrar a posição reportada aos poucos e desviar a aeronave sem que o
operador perceba; tomar o controle de um drone comercial dessa forma exige uma
manipulação gradual e cuidadosa dos sinais (Sathaye et al., 2022).

Detectores leves a bordo já existem: comparações entre IMU e GPS (Feng et al.,
2018; Kwon e Shim, 2020), invariantes físicos e de controle com CUSUM (Choi et
al., 2018; Quinonez et al., 2020) e os próprios testes de inovação do EKF no
PX4/ArduPilot. Só que eles rodam **acoplados ao estimador de navegação que o
spoofing contamina**: Jung et al. (2024) mostram ataques ao PX4 que mantêm a razão
de teste de inovação do EKF abaixo do limiar de detecção. Do outro lado, a
literatura de aprendizado de máquina (XGBoost, ensembles, redes neurais) relata
acurácias altas em conjuntos de dados, mas costuma avaliar os modelos offline ou
na estação de solo e raramente informa latência e memória em microcontrolador.

**Lacuna.** Falta caracterizar, para sUAS de baixo custo, um **monitor
independente em microcontrolador** (ESP32), desacoplado do controlador de voo:
quanto atraso e quantos falsos positivos ele tem, quanto custa por amostra e
**até que taxa de deriva** ele consegue detectar.

## Pergunta de pesquisa

Regras leves de plausibilidade física (velocidade/aceleração implícitas)
combinadas com consistência GPS × dead reckoning, num monitor ESP32 independente
do controlador de voo, detectam spoofing por salto e por deriva com baixo atraso e
FPR ≈ 0? Qual é a menor taxa de deriva detectável em função da janela de
re-ancoragem e do viés da velocidade inercial, e quanto isso custa (µs e bytes)
por amostra?

### Trabalhos relacionados (resumo)

| Trabalho | Ideia central | Relação com este projeto |
|---|---|---|
| Feng et al., ACM TECS 2018 | detecção de sequestro com giroscópio + GPS a bordo | consistência inercial × GPS sem hardware extra |
| Kwon e Shim, Sensors 2020 | aceleração do GPS × AHRS; mede a aceleração mínima de spoofing detectável | inspira a métrica de deriva mínima detectável |
| Choi et al., ACM CCS 2018 | invariantes de controle do veículo | física como referência, com overhead baixo |
| Quinonez et al. (SAVIOR), USENIX Security 2020 | invariantes físicos + CUSUM no PX4 | detecção a bordo de ataques furtivos |
| Tanıl et al., IEEE TAES 2018 | monitor INS sobre as inovações do filtro de Kalman contra spoofers que rastreiam a posição | base teórica; assume IMU de aeronave, não MEMS barata |
| Jung et al., IEEE Access 2024 | spoofing no PX4 abaixo do limiar de inovação do EKF | motiva um monitor independente do estimador |
| Sathaye et al., USENIX Security 2022 | tomada de controle de drones comerciais por spoofing | modelo de ameaça: deriva gradual |
| Sung et al., Sensors 2022; Ren et al., Future Internet 2023 | CNN 1D e destilação de conhecimento para rodar embarcado | alternativa por ML, mais pesada |

As referências completas estão no fim deste arquivo.

## Arquitetura

```
+------------------------- drone: ESP32 simulado (src/drone_sim.py) ---------------------+
|                                                                                        |
|  voo real --+--> GPS simulado --------+  (spoofing jump/drift só no GPS)               |
|             |                         +--> EdgeSpoofDetector --> flags, score, alerta  |
|             +--> IMU simulada --------+    (src/edge_detector.py)          |           |
|                                                                            v           |
|          paho-mqtt: telemetry (QoS 0) | alerts (QoS 1) | status (retido + Last Will)   |
|                     groundtruth (QoS 0, só avaliação)                                  |
+-------------------------------------------+-------------^------------------------------+
                                            |             |
                         MQTT 3.1.1, TCP 1883             | cmd/spoof (QoS 1)
                                            v             |
                    +----------------- Mosquitto (VM) -----------------+
                    |  lab/drone01/telemetry   lab/drone01/alerts      |
                    |  lab/drone01/status      lab/drone01/groundtruth |
                    |  lab/drone01/cmd/spoof                           |
                    +---------+----------------------------+-----------+
                              |                            |
                              v                            v
              +-------------------------------+  +-------------------------------+
              | MQTT X (host) ou MQTTX CLI    |  | estação de solo               |
              | assina lab/drone01/#          |  | src/ground_station.py         |
              | publica cmd/spoof (ataque)    |  | imprime e grava               |
              +-------------------------------+  | results/session_<ts>.csv      |
                                                 +-------------------------------+
```

## O detector (`src/edge_detector.py`)

| Checagem | Métrica por amostra | Limiar (`src/config.py`) |
|---|---|---|
| `speed` | velocidade implícita entre fixes GPS consecutivos | `V_MAX_MPS = 25` m/s |
| `accel` | variação do vetor velocidade implícito (pega também mudança brusca de direção) | `A_MAX_MPS2 = 8` m/s² |
| `dr_divergence` | distância entre a posição GPS e o dead reckoning da IMU | `D_MAX_M = 25` m |

- Uma amostra é **suspeita** se qualquer flag estiver ativa. O **alerta** só sai
  após `K_CONSECUTIVE = 2` amostras suspeitas seguidas, o que filtra picos isolados.
- **Dead reckoning:** integra a velocidade da IMU (regra do trapézio) a partir de
  uma âncora. A âncora volta para a posição GPS a cada `RESYNC_S = 30` s, mas
  **só se a amostra atual não for suspeita**. Durante um ataque a âncora congela,
  a divergência continua crescendo e o alerta se mantém.
- `score` é a maior razão métrica/limiar (≥ 1 indica que alguma checagem
  estourou). `alert_type` lista as checagens ativas no alerta, por exemplo
  `accel+dr_divergence`.
- `update()` devolve `DetectionResult(flags, score, alert, alert_type, details)`;
  `details` traz as métricas brutas, que vão no payload do alerta.
- **Piso de detecção:** uma deriva a `v` m/s acumula no máximo `v × RESYNC_S`
  metros entre duas re-ancoragens, então a menor deriva detectável é
  ≈ `D_MAX / RESYNC_S` (0,83 m/s com os valores padrão). Janelas maiores baixam o
  piso, mas acumulam também o erro da velocidade inercial.
- **Estado:** cerca de 12 números (último fix, última velocidade implícita, última
  velocidade da IMU, posição de DR, instante da âncora e contador). Nada cresce com
  o tempo de voo.

## Modelo de simulação (`src/drone_sim.py`)

- **Voo:** rota de 5 waypoints (~400 × 450 m) em torno de (-23,5505, -46,6333), a
  8 m/s e 50 m de altitude, em loop. A aceleração é limitada a 2 m/s² (curvas
  suaves); a posição real é integrada a 10 Hz e amostrada a 1 Hz.
- **GPS:** posição real + erro com σ = 2 m por eixo, **correlacionado no tempo**
  (processo de Gauss-Markov de 1ª ordem, τ = 60 s), mais uma componente branca de
  0,5 m. Receptores reais têm erro absoluto de metros que varia devagar, com pouca
  variação entre fixes consecutivos. Com ruído branco puro de 2 m a 1 Hz, a
  aceleração implícita teria desvio de ~4,9 m/s² por eixo e a regra de 8 m/s²
  dispararia em ~26% das amostras.
- **IMU:** velocidade real + ruído branco σ = 0,3 m/s por eixo, mais um viés
  opcional (`--imu-bias`, padrão 0). **Premissa:** existe uma fonte de velocidade
  independente do GPS, como uma IMU auxiliada por fluxo óptico. Uma IMU MEMS
  integrada sem auxílio acumula dezenas de metros de erro em dezenas de segundos;
  a varredura de sensibilidade mostra o efeito do viés.
- **Ataques, só no GPS:** `jump` desloca a posição em 150–300 m numa direção
  aleatória e mantém o offset; `drift` faz o offset crescer a `drift_mps` numa
  direção fixa aleatória. A velocidade Doppler (`gps_speed`) acompanha a trajetória
  falsa, como faria um spoofer consistente.
- **Reprodutibilidade:** `--seed`. O ataque usa um gerador aleatório separado, então
  a mesma seed produz o mesmo ruído de sensor com e sem ataque.

## Tópicos MQTT e payloads

| Tópico | QoS | Retido | Sentido | Conteúdo |
|---|---|---|---|---|
| `lab/drone01/telemetry` | 0 | não | drone → | 1 Hz, JSON abaixo |
| `lab/drone01/alerts` | 1 | não | drone → | quando o detector dispara (início de cada alerta) |
| `lab/drone01/status` | 1 | sim | drone → | `online`; Last Will = `offline` |
| `lab/drone01/cmd/spoof` | 1 | não | → drone | `{"mode":"none\|jump\|drift","drift_mps":2.0}` |
| `lab/drone01/groundtruth` | 0 | não | drone → | `{"seq","spoof_active","mode"}`, só para avaliação |

Exemplos reais, capturados na demo:

```json
// lab/drone01/telemetry
{"id": "drone01", "seq": 47, "ts": 1794406167.449, "lat": -23.5510061, "lon": -46.6296898,
 "alt": 52.2, "gps_speed": 8.11, "imu_vn": -0.02, "imu_ve": 7.54, "battery": 99.1,
 "flags": {"speed": false, "accel": false, "dr_divergence": true},
 "score": 1.29, "alert": true, "det_us": 84.0}

// lab/drone01/alerts
{"seq": 46, "ts": 1794406166.449, "type": "dr_divergence", "score": 1.223,
 "details": {"implied_speed_mps": 8.47, "implied_accel_mps2": 2.77,
             "dr_divergence_m": 30.56, "consecutive": 2}}

// lab/drone01/groundtruth
{"seq": 47, "spoof_active": true, "mode": "drift"}
```

Na telemetria, `alert` (estado atual do alerta) e `det_us` (tempo do detector
nesta amostra) complementam os campos mínimos; o ground truth traz também `mode`.
O tópico `groundtruth` existe só para avaliar o detector e não teria equivalente
num drone real.

## Instalação

**Ubuntu 18.04** (a VM deste projeto). O Python do sistema é 3.6 e o paho-mqtt 2.x
exige 3.7 ou mais, por isso entra o Python 3.8 do próprio Ubuntu:

```bash
sudo apt update && sudo apt install -y mosquitto mosquitto-clients python3.8 python3.8-venv python3-distutils
python3.8 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/pip install -r requirements.txt
```

**Ubuntu 22.04/24.04:**

```bash
sudo apt update && sudo apt install -y mosquitto mosquitto-clients python3-venv
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

**Broker.** O pacote `mosquitto` já sobe um serviço do sistema. No 18.04
(Mosquitto 1.4.15) esse serviço escuta em todas as interfaces e sem autenticação;
no 22.04 ou mais novo (Mosquitto 2.x) escuta só em `localhost`. Para usar a
configuração do projeto (`config/mosquitto.conf`, com `listener 1883 0.0.0.0` e
`allow_anonymous true`):

```bash
sudo systemctl stop mosquitto      # ou disable --now, para não subir no boot
mosquitto -c config/mosquitto.conf -v
```

> **Segurança.** Esta é uma configuração de laboratório. Em produção: TLS na porta
> 8883 ou mTLS (`require_certificate true`), `allow_anonymous false` com
> `password_file`, ACL por tópico (o drone só publica nos próprios tópicos e
> `cmd/` fica restrito ao operador) e nenhum listener em texto claro exposto fora
> do host. O Mosquitto 1.4 do Ubuntu 18.04 só fala MQTT 3.1.1.

## Como rodar

### Opção A: um terminal

```bash
./scripts/run_demo.sh --attack drift --attack-start 20 --duration 60
```

Reaproveita o broker que já estiver na porta 1883 (ou sobe um com
`config/mosquitto.conf`), roda a estação de solo em segundo plano e o drone em
modo `--quiet`. Ctrl+C encerra tudo, e o drone publica `offline` antes de sair. Sem
`--attack`, o ataque pode ser disparado pelo MQTT X.

### Opção B: três terminais

```bash
# 1) broker (pule se o serviço do sistema estiver ativo)
mosquitto -c config/mosquitto.conf -v

# 2) estação de solo
.venv/bin/python src/ground_station.py

# 3) drone (ESP32 simulado)
.venv/bin/python src/drone_sim.py --attack drift --attack-start 20
```

| Opção do `drone_sim.py` | Padrão | Descrição |
|---|---|---|
| `--host`, `--port` | `localhost`, `1883` (ou `MQTT_HOST`/`MQTT_PORT`) | broker |
| `--attack {none,jump,drift}` | `none` | ataque agendado |
| `--attack-start N` | `60` | segundos até o ataque agendado |
| `--drift-mps V` | `2.0` | taxa da deriva (m/s) |
| `--duration S` | `0` (sem fim) | encerra após S segundos |
| `--seed N` | aleatória | reprodutibilidade |
| `--imu-bias σ` | `0` | desvio do viés da velocidade inercial (m/s) |
| `--quiet` | desligado | imprime só eventos (ataques, alertas, conexão) |

A estação de solo aceita `--host`, `--port`, `--csv`, `--quiet` e `--no-color`, e
grava `results/session_<timestamp>.csv` com uma linha por telemetria, já com o
`spoof_active` do ground truth.

Robustez: o drone reconecta sozinho se o broker cair (os alertas QoS 1 ficam na
fila do cliente e saem na reconexão), ignora comandos inválidos em `cmd/spoof` e,
com Ctrl+C ou SIGTERM, publica `offline` e desconecta de forma limpa. Se o processo
morrer sem desconectar, o broker publica o Last Will `offline`.

## Disparando o ataque pelo MQTT X

Conecte o MQTT X ao IP da VM (porta 1883, **MQTT 3.1.1** com o Mosquitto 1.4),
assine `lab/drone01/#` e publique em `lab/drone01/cmd/spoof`:

```json
{"mode":"drift","drift_mps":2.0}
```

Para encerrar o ataque, publique `{"mode":"none"}`. O passo a passo completo, com
espaço para as capturas de tela e os comandos do MQTTX CLI, está em
[docs/mqttx.md](docs/mqttx.md).

## Avaliação offline

```bash
.venv/bin/python scripts/evaluate.py
```

Roda o simulador e o detector importando as classes, sem broker (~15 s nesta VM).

- **Cenários:** sem ataque, salto, deriva a 0,5 / 1 / 2 / 4 m/s. São 20 seeds por
  cenário, 300 s por execução e ataque a partir de t = 100 s.
- **Comparação pareada:** "só speed+accel" e "speed+accel+dr_divergence" processam
  exatamente as mesmas amostras.
- **Fase da re-ancoragem:** sorteada em cada seed. Num voo real o ataque começa em
  qualquer ponto da janela de 30 s; sem o sorteio, todas as execuções teriam a mesma
  fase e o atraso medido ficaria enviesado.
- **Métricas:**
  - **TPR:** fração das execuções com alerta durante o ataque.
  - **FPR:** amostras em alerta sem ataque ÷ amostras sem ataque.
  - **Atraso:** do início do ataque até o primeiro alerta.
  - **Cobertura:** fração das amostras sob ataque que ficaram em alerta.
  - **Custo do detector:** em µs por amostra.
- **Saídas** em `results/`:
  - `metrics.csv` e `detection_delay.png`: avaliação principal;
  - `sensitivity.csv` e `drift_floor.png`: varredura de sensibilidade.
- **Opções:** `--seeds`, `--duration`, `--attack-start`, `--sweep-seeds` e `--no-sweep`.

### Resultados

Gerados pelo `scripts/evaluate.py` com os parâmetros padrão (seeds 0–19, Python 3.8
nesta VM). Em todos os cenários, o FPR foi medido sobre as amostras sem ataque.

| Cenário | Variante | TPR | FPR | Atraso mediano (s) | Atraso p95 / máx (s) | Cobertura |
|---|---|---|---|---|---|---|
| sem ataque | speed+accel | — | 0,00% (0/20 execuções) | — | — | — |
| sem ataque | speed+accel+dr_divergence | — | 0,00% (0/20 execuções) | — | — | — |
| salto 150–300 m | speed+accel | 100% | 0,00% | 1,0 | 1,0 / 1,0 | 0,5% |
| salto 150–300 m | speed+accel+dr_divergence | 100% | 0,00% | 1,0 | 1,0 / 1,0 | 99,5% |
| deriva 0,5 m/s | speed+accel | 0% | 0,00% | — | — | 0% |
| deriva 0,5 m/s | speed+accel+dr_divergence | 0% | 0,00% | — | — | 0% |
| deriva 1 m/s | speed+accel | 0% | 0,00% | — | — | 0% |
| deriva 1 m/s | speed+accel+dr_divergence | 100% | 0,00% | 39,5 | 47,0 / 50,0 | 81,3% |
| deriva 2 m/s | speed+accel | 0% | 0,00% | — | — | 0% |
| deriva 2 m/s | speed+accel+dr_divergence | 100% | 0,00% | 14,0 | 26,0 / 26,0 | 92,0% |
| deriva 4 m/s | speed+accel | 0% | 0,00% | — | — | 0% |
| deriva 4 m/s | speed+accel+dr_divergence | 100% | 0,00% | 7,0 | 12,0 / 13,0 | 96,1% |

**Varredura de sensibilidade.** Menor deriva detectada com TPR ≥ 90%, com 10 seeds
por ponto e taxas testadas de 0,25 a 4 m/s; "inviável" significa FPR sem ataque
acima de 1%.

| Viés da IMU (σ por eixo) | RESYNC_S = 10 s | 20 s | 30 s | 60 s |
|---|---|---|---|---|
| 0 | 2,8 m/s | 1,4 m/s | 1,0 m/s | 0,5 m/s |
| 0,1 m/s | 2,8 m/s | 1,4 m/s | 1,0 m/s | 0,5 m/s |
| 0,25 m/s | 2,8 m/s | 1,4 m/s | 1,4 m/s | inviável (FPR 23,5%) |
| 0,5 m/s | 2,8 m/s | inviável (FPR 9,1%) | inviável (FPR 29,8%) | inviável (FPR 79,0%) |
| piso teórico `D_MAX/RESYNC_S` | 2,50 m/s | 1,25 m/s | 0,83 m/s | 0,42 m/s |

Os valores medidos caem na grade de taxas testadas (0,25 / 0,35 / 0,5 / 0,7 / 1 /
1,4 / 2 / 2,8 / 4 m/s), por isso ficam logo acima do piso teórico.

**O que os números dizem:**

1. **Salto:** as duas variantes detectam 100% em 1 s, com FPR zero. Mas, só com
   speed+accel, o alerta dura uma amostra (cobertura de 0,5%): o GPS continua falso
   e o detector "esquece". Com o dead reckoning, a âncora congelada mantém o alerta
   durante todo o ataque (cobertura de 99,5%).
2. **Deriva:** as regras de salto nunca detectam deriva (0% em todas as taxas).
   Com `dr_divergence` e `RESYNC_S = 30 s`, a detecção é de 100% a partir de 1 m/s;
   0,5 m/s fica abaixo do piso de 0,83 m/s e passa sem alerta, como prevê o modelo.
3. **Piso:** a menor deriva detectável acompanha `D_MAX/RESYNC_S`. Janelas longas
   baixam o piso, mas só funcionam com velocidade inercial de baixo viés: com σ de
   0,25 m/s a janela de 60 s gera 23,5% de FPR, e com 0,5 m/s só a janela de 10 s
   (piso de 2,8 m/s) é viável. A qualidade da referência de velocidade é o
   parâmetro que decide até onde o monitor enxerga a deriva lenta.
4. **Custo:** ~13 µs por amostra em CPython nesta VM, com estado de tamanho
   constante (seção abaixo).

### Testes

```bash
.venv/bin/python -m pytest -q
```

São 22 testes:
- haversine, rumo e deslocamento;
- sem ataque, nenhum alerta em 300 s (5 seeds);
- salto gera alerta em até 3 s e o alerta se mantém;
- deriva de 2 m/s é pega pela `dr_divergence` e não pela regra `speed` sozinha;
- um pico isolado de aceleração não alerta com K = 2;
- o ataque altera o GPS sem afetar a IMU;
- validação dos comandos de `cmd/spoof`.

## Viabilidade no ESP32

- **Medido (CPython 3.8 nesta VM):** na avaliação offline, o detector completo leva
  em média 13,4 µs por amostra (p99 de 35,5 µs, em 36.000 amostras). No laço de
  tempo real a 1 Hz da demo, com cache frio entre amostras, ficou em ~82 µs
  (p99 de ~166 µs). Os dois valores estão ordens de grandeza abaixo do período de
  1 s. O `drone_sim.py` publica esse tempo em `det_us` e resume as estatísticas ao
  encerrar.
- **Custo por amostra:** 2 haversines, 1 rumo e 1 deslocamento (cerca de 15
  funções trigonométricas e raízes) mais algumas dezenas de operações aritméticas;
  sem alocação dinâmica.
- **Memória:** cerca de 12 números de estado mais 6 parâmetros, ou seja, menos de
  200 bytes em `double`.
- **Estimativa para o ESP32 (a confirmar no hardware):** o ESP32 clássico e o S3
  têm FPU só de precisão simples (o C3 não tem FPU), então `double` é emulado em
  software. Mesmo assim, a ordem de grandeza esperada em C é de dezenas a poucas
  centenas de µs por amostra, menos de 0,1% do período de 1 s. A medição no
  dispositivo deve usar `esp_timer_get_time()` no ESP-IDF ou `time.ticks_us()` no
  MicroPython.
- **Porte:** no MicroPython padrão do ESP32 o `float` tem 32 bits, o que dá
  resolução de ~0,2–0,4 m em lat/lon nesta região. Vale converter para
  coordenadas locais (leste/norte em metros, relativas a uma referência) e
  trabalhar em `float32`; a aproximação equiretangular de `geo.offset_latlon` já
  segue esse caminho.

## Limitações

- **Tudo é simulado:** voo, sensores e ataques. Os modelos de ruído são plausíveis,
  mas não foram calibrados com logs reais.
- **Premissa da velocidade inercial:** o modelo padrão assume uma fonte de
  velocidade independente do GPS e com erro limitado. Uma IMU MEMS pura, integrada
  sem auxílio, exigiria janelas curtas e teria piso alto (ver a varredura).
- **Limiares fixos** (`V_MAX`, `A_MAX`, `D_MAX`, `RESYNC_S`, `K`), escolhidos para
  este perfil de voo (8 m/s, curvas suaves).
- **Spoofer adaptativo:** um atacante que conheça os limiares pode derivar abaixo
  do piso (`D_MAX/RESYNC_S`) sem ser detectado. Isso é inerente a qualquer janela
  de consistência e é justamente o que a varredura quantifica.
- **Sem recuperação automática:** depois do alerta, a âncora fica congelada. Com
  viés alto na IMU, um alarme falso pode não se encerrar sozinho; num sistema real
  isso pede uma política de recuperação (confirmação do operador, por exemplo).
- **Escopo do detector:** o detector roda só a 1 Hz, junto com o fix GPS, num
  único drone, e não trata lacunas longas de GPS (por exemplo, jamming seguido de
  spoofing).
- **Segurança do MQTT:** sem TLS e sem autenticação, e o `cmd/spoof` aceita
  comandos de qualquer cliente. É adequado só para laboratório.
- **Broker antigo:** o Mosquitto 1.4 do Ubuntu 18.04 só aceita MQTT 3.1.1.

## Trabalhos futuros

- Portar o detector para um ESP32 real (C/ESP-IDF ou MicroPython), com GPS u-blox
  e IMU reais (mais fluxo óptico), e medir µs e bytes no dispositivo.
- TLS/mTLS no Mosquitto, autenticação e ACL por tópico.
- Assinatura das mensagens (HMAC ou Ed25519) para garantir integridade e origem de
  telemetria, alertas e comandos.
- Limiares adaptativos, por exemplo a partir da precisão reportada pelo receptor
  (HDOP/hAcc) e da qualidade estimada da IMU.
- Teste sequencial (CUSUM) sobre a divergência e re-ancoragem condicionada, para
  baixar o piso de detecção.
- Combinar com os indicadores de spoofing do próprio receptor (u-blox) e com C/N0.
- Avaliar com logs reais de voo (PX4/ArduPilot) e com atacantes furtivos como os de
  Jung et al. (2024).

## Estrutura do repositório

```
esp32-gps-spoofing-edge-detector/
├── config/mosquitto.conf       # broker de laboratório (0.0.0.0:1883, anônimo)
├── src/
│   ├── config.py               # broker, tópicos, parâmetros da simulação e limiares
│   ├── geo.py                  # haversine, rumo, deslocamento lat/lon por metros
│   ├── edge_detector.py        # detector de borda (puro, sem MQTT)
│   ├── drone_sim.py            # firmware simulado do ESP32 + núcleo da simulação
│   └── ground_station.py       # assina os tópicos, imprime e grava CSV
├── scripts/
│   ├── run_demo.sh             # broker + estação de solo + drone num terminal
│   └── evaluate.py             # avaliação offline, métricas e gráficos
├── tests/test_edge_detector.py
├── results/                    # saídas geradas (CSV/PNG ignorados pelo git)
├── docs/mqttx.md               # passo a passo do MQTT X, com espaço para prints
├── requirements.txt
├── LICENSE
└── README.md
```

## Referências

1. Feng, Z.; Guan, N.; Lv, M.; Liu, W.; Deng, Q.; Liu, X.; Yi, W. An Efficient UAV
   Hijacking Detection Method Using Onboard Inertial Measurement Unit. *ACM
   Transactions on Embedded Computing Systems*, 17(6), art. 96, 2018.
   <https://doi.org/10.1145/3289390>
2. Kwon, K.-C.; Shim, D.-S. Performance Analysis of Direct GPS Spoofing Detection
   Method with AHRS/Accelerometer. *Sensors*, 20(4), 954, 2020.
   <https://doi.org/10.3390/s20040954>
3. Choi, H.; Lee, W.-C.; Aafer, Y.; Fei, F.; Tu, Z.; Zhang, X.; Xu, D.; Deng, X.
   Detecting Attacks Against Robotic Vehicles: A Control Invariant Approach.
   *ACM CCS*, p. 801–816, 2018.
4. Quinonez, R.; Giraldo, J.; Salazar, L.; Bauman, E.; Cardenas, A.; Lin, Z.
   SAVIOR: Securing Autonomous Vehicles with Robust Physical Invariants. *USENIX
   Security Symposium*, 2020.
   <https://www.usenix.org/conference/usenixsecurity20/presentation/quinonez>
5. Tanıl, Ç.; Khanafseh, S.; Joerger, M.; Pervan, B. An INS Monitor to Detect GNSS
   Spoofers Capable of Tracking Vehicle Position. *IEEE Transactions on Aerospace
   and Electronic Systems*, 2018.
6. Jung, Hong, Choi e Yoon. An Analysis of GPS Spoofing Attack and Efficient
   Approach to Spoofing Detection in PX4. *IEEE Access*, 12, p. 46668–46677, 2024.
   <https://doi.org/10.1109/ACCESS.2024.3382543>
7. Sathaye, H.; Strohmeier, M.; Lenders, V.; Ranganathan, A. An Experimental Study
   of GPS Spoofing and Takeover Attacks on UAVs. *USENIX Security Symposium*, 2022.
   <https://www.usenix.org/conference/usenixsecurity22/presentation/sathaye>
8. Sung et al. GPS Spoofing Detection Method for Small UAVs Using 1D Convolution
   Neural Network. *Sensors*, 22(23), 9412, 2022.
   <https://www.mdpi.com/1424-8220/22/23/9412>
9. Ren et al. Knowledge Distillation-Based GPS Spoofing Detection for Small UAV.
   *Future Internet*, 15(12), 389, 2023. <https://www.mdpi.com/1999-5903/15/12/389>

## Autor

Floriano Neto

## Licença

MIT. Veja [LICENSE](LICENSE).
