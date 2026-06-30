# Seguidor de Linha — Grupo B1 (ES670)

Robô seguidor de linha com STM32G474 (Nucleo-G474RE), FreeRTOS, sensores IR, LCD I2C, Bluetooth HC-05 e ultrassom HC-SR04.

## Status atual

O robô **segue a linha de forma razoável**, mas ainda **sai na largada** em alguns testes. A calibração do PID e da velocidade base precisa de ajuste fino no pista real. Use o Bluetooth para tunar sem recompilar (veja seção abaixo).

**Problema conhecido:** nos primeiros ~500 ms o erro do sensor pode oscilar (spread alto na largada), gerando correção forte. Já existe rampa de correção e zona morta, mas os ganhos (`Kp`, `SPD`) ainda precisam ser afinados para o seu chão e fita.

---

## O que foi implementado

### Seguir linha (em progresso)

| Item | Status |
|------|--------|
| Máquina de estados (IDLE → ALIGN → CALIB → FOLLOW) | OK |
| PID de linha (proporcional, sinal corrigido) | OK, precisa tuning |
| Leitura 5 sensores IR via ADC | OK |
| Busca quando perde a linha (pivot) | OK |
| Passagem por cruzamento (`STATE_IN_CROSSING`, 300 ms reto) | OK |
| Parada por timeout / fim de percurso | OK |
| Parada por obstáculo (ultrassom) | OK |

### Novos módulos

| Arquivo | Função |
|---------|--------|
| `telemetry.c/h` | Dados compartilhados (distância, velocidade, heading, bateria) com mutex FreeRTOS |
| `lcd_i2c.c/h` | LCD 16x2 via PCF8574 (I2C2), atualização 1 Hz |
| `bluetooth.c/h` | HC-05 (USART3): monitoramento, configuração e controle manual |
| `ultrasonic.c/h` | HC-SR04: distância + parada de segurança + buzzer |

### Correções importantes já feitas

1. **PID invertido** — `vel_left = base - correction` (antes corrigia para o lado errado).
2. **Busca invertida** — ao perder a linha, gira na direção onde ela foi vista por último.
3. **Prioridades FreeRTOS** — `LineCtrlTask` = Normal, `MotorCtrlTask` = AboveNormal.
4. **Fila de motores** — `sizeof(MotorCmd_t)`, não `uint32_t`.
5. **Erro do sensor** — centroide com zona morta e filtro de spread alto na largada.
6. **Rampa de correção** — primeiros 500 ms com correção gradual.

### Arquitetura FreeRTOS

```
MotorCtrlTask   (100 Hz, AboveNormal) — motores, encoder, telemetria
LineCtrlTask    ( 50 Hz, Normal)      — sensores, PID, ultrassom, estados
LcdTask         (  1 Hz, BelowNormal) — LCD 16x2
BluetoothTask   ( 20 Hz, BelowNormal) — HC-05 UART
```

Recursos: `osMessageQueue` (comandos motor), `osMutex` (telemetria), `vTaskDelayUntil` em todas as tasks.

---

## Como rodar o projeto

### 1. Clonar e abrir

```bash
git clone https://github.com/raissabbsa/ES670-2026-GrupoB1.git
cd ES670-2026-GrupoB1
```

1. Abra o **STM32CubeIDE**.
2. **File → Open Projects from File System** → pasta `Projeto Final`.
3. Abra `Projeto Final.ioc` e clique em **Generate Code** (cria `Drivers/` e `Middlewares/` localmente — **não commitar**).
4. **Project → Clean** → **Project → Build All**.

> Se o linker reclamar de `.c` faltando, confira se `telemetry.c`, `lcd_i2c.c`, `bluetooth.c` e `ultrasonic.c` estão no build (Generate Code costuma incluir arquivos em `Core/Src` automaticamente).

### 2. Gravar e testar na placa

1. Conecte a Nucleo via USB.
2. **Run → Debug** (ou Flash).
3. Serial de debug: **LPUART1**, **115200 baud** (minicom, PuTTY, etc.).

### 3. Controles na placa

| Ação | Botão |
|------|-------|
| Iniciar (alinha + calibra + segue) | **Enter** |
| Modo debug dos sensores IR | **Baixo** (toggle) |
| Parar durante seguimento | **Switch frontal** |

### 4. Saída serial esperada (debug)

```
Robo OK - serial 115200
Enter=inicia (alinha sozinho)
=== SEGUINDO LINHA ===
F ON vl=35 vr=35 adj=0 spr=55
```

- `adj` = erro filtrado × 100 (0 = no centro).
- `spr` = contraste entre sensores (quanto maior, mais “confuso” o instante).
- `BUSCA` = linha perdida, procurando.

---

## Ajustar sem recompilar (Bluetooth)

1. Pareie o HC-05 no celular (PIN padrão costuma ser `1234` ou `0000`).
2. Instale **Serial Bluetooth Terminal** (Android) ou similar.
3. Conecte ao módulo e envie comandos (uma linha por comando):

### Tunagem do seguidor

```
$SET,KP,0.45
$SET,KI,0.00
$SET,KD,0.00
$SET,SPD,0.35
```

Resposta: `$OK`

| Parâmetro | Efeito | Valores atuais no código |
|-----------|--------|--------------------------|
| `KP` | Força da correção na curva | 0.45 (subir = corrige mais, pode oscilar) |
| `KI` | Corrige erro persistente | 0.0 |
| `KD` | Amortecimento | 0.0 |
| `SPD` | Velocidade base (0.0–1.0) | 0.35 |

**Sugestão para quem continuar o tuning:**

- Se **sai da linha na largada**: baixe `KP` (`0.35`) e `SPD` (`0.30`).
- Se **não corrige curva**: suba `KP` aos poucos (`0.50`, `0.55`).
- Se **oscila (zigue-zague)**: baixe `KP` ou suba zona morta no código (`line_sensor.c`).

### Monitoramento (1 Hz automático)

O robô envia:

```
$MON,dist_cm,vel_cms,heading_deg,bat_pct,obst_cm,state
```

### Controle manual

```
$CMD,FWD
$CMD,REV
$CMD,LEFT
$CMD,RIGHT
$CMD,STOP
$CMD,START
```

---

## O que NÃO commitar

Gerados localmente pelo CubeMX / build — já estão no `.gitignore`:

- `Projeto Final/Drivers/`
- `Projeto Final/Middlewares/`
- `Projeto Final/Debug/`
- `.metadata/`, `.settings/`, `.cproject`, `.project`

Cada pessoa roda **Generate Code** no `.ioc` após o `git pull`.

---

## Estrutura do código principal

```
Projeto Final/Core/Src/
├── app_tasks.c      ← máquina de estados + PID + seguir linha
├── line_sensor.c    ← leitura ADC + centroide + cruzamento
├── motor.c          ← PWM TIM1
├── pid.c            ← controlador PID
├── encoder.c        ← velocidade / distância
├── telemetry.c      ← dados compartilhados
├── lcd_i2c.c        ← display
├── bluetooth.c      ← HC-05
├── ultrasonic.c     ← HC-SR04 + segurança
└── main.c           ← init + botões + tasks FreeRTOS
```

---

## Próximos passos (para o grupo)

- [ ] Afinar `Kp` e `base_speed` na pista de prova
- [ ] Validar parada correta no fim do percurso
- [ ] Confirmar cruzamento em T
- [ ] Tempo total < 1 min 30 s
- [ ] Testar LCD e telemetria Bluetooth com hardware montado
- [ ] Documentar ganhos finais que funcionaram

---

## Disciplina

ES670 — Projeto Final — Grupo B1
