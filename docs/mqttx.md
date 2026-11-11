# Demo com o MQTT X

Passo a passo para acompanhar a telemetria e os alertas do drone e para disparar
ataques de spoofing pelo MQTT X (aplicativo desktop ou CLI).

> Os marcadores **📸 PRINT N** indicam onde inserir capturas de tela. Sugestão:
> salvar as imagens em `docs/img/` e trocar cada marcador por
> `![descrição](img/NN-nome.png)`.

## 0. Pré-requisitos

- Broker rodando na VM e escutando em `0.0.0.0:1883`: o serviço `mosquitto` do
  sistema ou `mosquitto -c config/mosquitto.conf -v` (veja o README).
- Drone e estação de solo rodando: `./scripts/run_demo.sh` ou os três terminais
  descritos no README. Para disparar o ataque só pelo MQTT X, rode o drone sem
  `--attack`.
- IP da VM: `hostname -I` (por exemplo, `192.168.16.133`).
- O host precisa alcançar a VM na porta 1883:
  - VMware (NAT ou host-only) e modo bridge costumam funcionar direto com o IP da VM;
  - VirtualBox em NAT: crie um redirecionamento de porta 1883 → 1883 e use
    `127.0.0.1` como host no MQTT X;
  - firewall ativo na VM: `sudo ufw allow 1883/tcp`;
  - teste no host: `nc -vz <IP_DA_VM> 1883` (Linux/macOS) ou
    `Test-NetConnection <IP_DA_VM> -Port 1883` (PowerShell).

> **Versão do protocolo.** O Mosquitto do Ubuntu 18.04 (1.4.15) só aceita
> MQTT 3.1.1, e o MQTT X usa MQTT 5.0 por padrão: selecione **3.1.1** na
> conexão. Com Mosquitto 2.x (Ubuntu 22.04/24.04) o 5.0 também funciona.

## 1. Criar a conexão (MQTT X desktop, no host)

1. Clique em **+ New Connection**.
2. Preencha:
   - **Name:** `lab-drone01`
   - **Host:** `mqtt://` e o IP da VM (ex.: `192.168.16.133`)
   - **Port:** `1883`
   - **Client ID:** pode manter o gerado automaticamente
   - **Username/Password:** em branco (laboratório sem autenticação)
   - **SSL/TLS:** desligado
3. Em **Advanced**, ajuste **MQTT Version** para **3.1.1** (necessário com o Mosquitto 1.4).
4. Clique em **Connect**.

📸 **PRINT 1** — tela "New Connection" preenchida (host, porta e MQTT 3.1.1).

## 2. Assinar os tópicos do drone

1. Clique em **+ New Subscription**.
2. **Topic:** `lab/drone01/#` · **QoS:** 1 · **Confirm**.
3. Devem aparecer:
   - `lab/drone01/status` com `online` (mensagem retida: chega assim que você assina);
   - `lab/drone01/telemetry` a cada 1 s (JSON com `flags`, `score` e `alert`);
   - `lab/drone01/groundtruth` a cada 1 s (`spoof_active`, só para avaliação).
4. Dica: crie uma segunda assinatura só para `lab/drone01/alerts`, com outra cor,
   para os alertas se destacarem no meio da telemetria.

📸 **PRINT 2** — telemetria chegando (payload JSON formatado) e status `online`.

## 3. Disparar o ataque de deriva

1. Na área de publicação (parte de baixo da janela):
   - **Topic:** `lab/drone01/cmd/spoof`
   - **QoS:** 1 · **Retain:** desmarcado · formato do payload: **JSON**
   - **Payload:**

     ```json
     {"mode":"drift","drift_mps":2.0}
     ```

2. Envie. O terminal do drone mostra
   `ataque no GPS -> drift (2.0 m/s) [origem: MQTT lab/drone01/cmd/spoof]`, e
   `groundtruth` passa a `"spoof_active": true`.
3. Em cerca de 11 a 26 s (mediana de 14 s na avaliação), dependendo de onde o
   ataque cai na janela de re-ancoragem de 30 s, chega uma mensagem em
   `lab/drone01/alerts`. Exemplo real, capturado na demo:

   ```json
   {"seq": 46, "ts": 1794406166.449, "type": "dr_divergence", "score": 1.223,
    "details": {"implied_speed_mps": 8.47, "implied_accel_mps2": 2.77,
                "dr_divergence_m": 30.56, "consecutive": 2}}
   ```

   Na telemetria, `flags.dr_divergence` e `alert` passam a `true`.

📸 **PRINT 3** — alerta recebido em `lab/drone01/alerts`.

## 4. Encerrar o ataque

1. Publique em `lab/drone01/cmd/spoof`:

   ```json
   {"mode":"none"}
   ```

2. O GPS volta à posição real. O retorno brusco ainda é um salto, então as
   regras de velocidade/aceleração mantêm o alerta por mais 1 ou 2 amostras;
   depois as flags zeram e `groundtruth` volta a `false`.

📸 **PRINT 4** — telemetria com as flags zeradas depois do `{"mode":"none"}`.

## 5. Outros testes rápidos

| Payload em `cmd/spoof` | O que esperar |
|---|---|
| `{"mode":"jump"}` | alerta em ~1 s, tipo `accel+dr_divergence` |
| `{"mode":"drift","drift_mps":4.0}` | alerta em ~6 a 13 s, tipo `dr_divergence` |
| `{"mode":"drift","drift_mps":0.5}` | **sem alerta**: abaixo do piso de ~0,83 m/s com `RESYNC_S = 30 s` (resultado esperado, ver README) |
| `{"mode":"teleport"}` | comando inválido: o drone registra o erro e ignora |

**Last Will:** encerre o drone com `kill -9 <pid>` e veja o broker publicar
`offline` em `lab/drone01/status` (na hora, porque o sistema fecha o socket; numa
queda de rede sem aviso, após ~1,5 × keepalive = 45 s). Com Ctrl+C o próprio drone
publica `offline` antes de desconectar.

📸 **PRINT 5** — `lab/drone01/status` mudando para `offline`.

## 6. VM sem interface gráfica: MQTTX CLI ou mosquitto-clients

MQTTX CLI (instalação: <https://mqttx.app/docs/cli/downloading-and-installation>),
no host ou na VM:

```bash
mqttx sub -h <IP_DA_VM> -p 1883 -t 'lab/drone01/#' -V 3.1.1
```

```bash
mqttx pub -h <IP_DA_VM> -p 1883 -t 'lab/drone01/cmd/spoof' -q 1 -V 3.1.1 -m '{"mode":"drift","drift_mps":2.0}'
```

```bash
mqttx pub -h <IP_DA_VM> -p 1883 -t 'lab/drone01/cmd/spoof' -q 1 -V 3.1.1 -m '{"mode":"none"}'
```

Alternativa já instalada na VM pelo pacote `mosquitto-clients`:

```bash
mosquitto_sub -h localhost -t 'lab/drone01/#' -v
```

```bash
mosquitto_pub -h localhost -t 'lab/drone01/cmd/spoof' -q 1 -m '{"mode":"drift","drift_mps":2.0}'
```

📸 **PRINT 6** (opcional) — terminal com `mqttx sub` mostrando um alerta.
