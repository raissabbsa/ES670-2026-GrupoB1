# Seguidor de Linha — Grupo B1 (ES670)

Robô seguidor de linha com STM32G474 (Nucleo-G474RE), FreeRTOS, 5 sensores IR, LCD I2C, Bluetooth HC-05, ultrassom HC-SR04 e encoders.

## Status do projeto (jul/2026)

### Pontuação (10 pontos no total)

| Item | Pontos | Status |
|------|--------|--------|
| (2,0) Seguir linha corretamente | 2,0 | OK |
| (0,5) Passar pelo cruzamento | 0,5 | OK |
| (0,5) Cumprir trajeto em ate 1:30 | 0,5 | OK |
| (0,5) Parar no fim do percurso | 0,5 | OK |
| (1,0) Monitoramento via celular (1Hz) | 1,0 | OK |
| (1,0) Configuracao via celular | 1,0 | OK |
| (0,5) Controle manual via celular | 0,5 | OK (car controller) |
| (1,0) Seguranca (obstaculo + colisao) | 1,0 | OK |
| (1,5) LCD 1Hz (D, V, H, Bat) | 1,5 | OK (depende de encoders) |
| (1,5) FreeRTOS | 1,5 | OK |

### O que foi implementado

**Seguir linha:**
- Maquina de estados IDLE -> ALIGNING -> CALIBRATING -> FOLLOWING
- PID adaptativo com max correction 0.18 (reta) e 0.30 (curva)
- Dead reckoning em linha perdida (estilo ES070, sem busca)
- Cruzamento: detecta `LINE_CROSSING` e anda reto 300ms
- Parada por timeout (90s) ou via botao de panico

**Calibracao automatica (sem chute manual):**
- `$CMD,CALIB,S` - calibracao passiva de sensores. O usuario move o robo manualmente sobre chao-fita-chao. O firmware captura min/max reais e detecta polaridade. Termina com `$CMD,CALIB,S,STOP` ou timeout de 30s.
- `$CMD,CALIB,M` - calibracao ativa de motores. O robo anda sozinho 2s para frente, para 200ms, anda 1s para tras, para 200ms. Encoder mede quanto cada roda andou. Calcula `ratio = L/R` e aplica LSCL/RSCL automaticamente.

**Bluetooth (HC-05, 9600 baud):**
- `$SET,KP/KI/KD/SPD/LSCL/RSCL` - configura parametros
- `$SET,POL,0|1|-1` - forca polaridade (DARK/LIGHT/automatico)
- `$SET,LCDDBG,1` - ativa modo debug no LCD (mostra MC, encoders, bateria raw)
- `$SET,DBGRAW,1` - ativa eco RAW de cada linha recebida (com echo na serial de debug)
- `$GET,PID` / `$GET,ALL` - le parametros
- `$GET,MON` - telemetria (D, V, H, Bat, obs, state)
- `$CMD,CALIB,S` / `$CMD,CALIB,M` - inicia calibracoes
- `$CMD,START` / `$CMD,STOP` - inicia/para seguimento
- `$CMD,FWD/REV/LEFT/RIGHT` - manual
- Car controller F/B/L/R/G/H/I/J/S (1 char) - Arduino Bluetooth Control

**Seguranca:**
- Ultrassom HC-SR04: le distancia a cada 50ms, com debounce de 3 leituras abaixo de 10cm antes de parar. Ativa buzzer.
- Botao de panico frontal (Switch_Fr, PD2): a cada ciclo do motor task, se pressionado, `Motor_Brake()` imediato e `STATE_STOPPING`.

**LCD (PCF8574 16x2, I2C2, 1Hz):**
- Modo normal: `D:NNcm V:NN` (distancia e velocidade), `H:NNN Bat:NN%` (heading e bateria)
- Modo debug (`$SET,LCDDBG,1`): `MC:NNNNN E:NNNN` (motor cycles + encoder left), `Bat:NNNN R:NNNN` (bateria raw + encoder right)

**FreeRTOS:**
- 5 tasks: LineCtrl (50Hz), MotorCtrl (100Hz), LCD (1Hz), BT (20Hz), e controle implicito via estado global
- 1 mutex (Telemetry)
- 1 queue (MotorCmd)
- vTaskDelayUntil em todas as tasks

### Comportamento de seguir linha

| Estado | Acao |
|--------|------|
| Reta (`|err|<0.15`) | Base 0.30, max correction 0.18 |
| Curva aberta (0.15-0.25) | Base 0.18, max correction 0.30 |
| Curva fechada (`|err|>0.30`) | Pivot (roda lenta para) |
| Linha perdida 100ms | Continua com ultimo erro (dead reckoning) |
| Linha perdida >2s | Para motores |

### Defaults no firmware

| Parametro | Valor | Descricao |
|-----------|-------|-----------|
| KP | 0.35 | Ganho proporcional |
| KI | 0.0 | Sem integrador (estavel) |
| KD | 0.04 | Derivativo com filtro passa-baixa |
| base_speed | 0.30 | Velocidade base em reta |
| LSCL | 0.95 | Escala motor esquerdo |
| RSCL | 1.20 | Escala motor direito |
| min sensor | 300 | Chao preto |
| max sensor | 1100 | Linha branca |

---

## Como rodar o projeto

### Compilar e gravar

1. Abra o **STM32CubeIDE**.
2. **File -> Open Projects from File System** -> pasta `Projeto Final`.
3. Abra `Projeto Final.ioc` e clique em **Generate Code** (cria `Drivers/` e `Middlewares/` localmente - nao commitar).
4. **Project -> Clean** -> **Project -> Build All**.
5. **Run -> Debug** (ou Flash) na placa Nucleo via USB.

### Conexao serial

- **LPUART1** (debug): 115200 baud - usa para ver log de calibracao, telemetria, mensagens de estado
- **USART3** (Bluetooth HC-05): 9600 baud - comandos do celular

### Botoes

| Acao | Botao |
|------|-------|
| Iniciar (alinha + calibra + segue) | **Enter** / **Cima** |
| Modo debug dos sensores IR | **Baixo** (toggle) |
| Parada de emergencia (brake imediato) | **Switch frontal** |

### Fluxo de uso recomendado

1. Ligue a placa. O log aparece no LPUART1.
2. **Calibre os sensores** (passivo, usuario move):
   - Mande `$CMD,CALIB,S` (LED pisca rapido)
   - Mova o robo sobre a pista (chao-fita-chao-fita-chao)
   - Mande `$CMD,CALIB,S,STOP`
   - Log: `Calib S OK: pol=LIGHT mean=850 m=[...]/[...]`
3. **Calibre os motores** (ativo, robo anda):
   - Coloque o robo em superficie plana, ~50cm de espaco
   - Mande `$CMD,CALIB,M`
   - Robo anda sozinho 3s
   - Log: `Calib M OK: L=265 R=320 ratio=0.83 LSCL=0.83 RSCL=1.00`
4. **Siga a linha**:
   - Coloque o robo sobre a fita
   - Aperte **Enter** ou mande `$CMD,START`
   - Log: `=== ALINHANDO ===` -> `=== CALIBRANDO (1.2s) ===` -> `=== SEGUINDO LINHA ===`

### Fluxo alternativo (calibracao manual)

Se a calibracao automatica nao funcionar, ajuste via Bluetooth:
- `$SET,LSCL,X` / `$SET,RSCL,X` - escala dos motores (padrao 1.0/1.0)
- `$SET,SPD,X` - velocidade base
- `$SET,KP,X` / `$SET,KI,X` / `$SET,KD,X` - parametros PID

Para `$SET,LSCL,1.10` se o robo puxa para esquerda, ou `$SET,RSCL,1.10` se puxa para direita.

### Diagnostico

Se algo nao funcionar:

1. **Mande `$SET,DBGRAW,1`** para ver cada linha recebida (com eco na serial de debug tambem).
2. **Mande `$SET,LCDDBG,1`** para ver:
   - `MC:NNNNN` (motor cycles, deve crescer ~100/s)
   - `E:NNNN` (encoder left, deve crescer quando a roda gira)
   - `Bat:NNNN` (ADC raw bateria, deve ser >100)
   - `R:NNNN` (encoder right)
3. **Mande `$GET,PID`** para ver parametros ativos.
4. **Mande `$GET,MON`** para ver telemetria.

### Arduino Bluetooth Control (app Android)

1. Instale o app **Arduino Bluetooth Control** (no Google Play).
2. Conecte ao HC-05 (PIN padrao 1234 ou 0000).
3. Va em **Car Controller** (ou configure botoes para enviar 1 caractere).
4. Configure os botoes:
   - Botao cima: envia `F` (frente)
   - Botao baixo: envia `B` (re)
   - Botao esquerda: envia `L`
   - Botao direita: envia `R`
5. Use tambem a aba **Terminal** para comandos completos: `$GET,PID`, `$SET,KP,0.30`, etc.

---

## Estrutura do codigo

```
Projeto Final/Core/
├── Inc/
│   ├── app_tasks.h         # estados, AppConfig_t, prototipos
│   ├── bluetooth.h         # buffer RX BT
│   ├── encoder.h           # TIM16/17 encoder
│   ├── lcd_i2c.h           # PCF8574 LCD
│   ├── line_sensor.h       # 5 sensores IR
│   ├── main.h              # pinagens, handles
│   ├── motor.h             # PWM TIM1 motores
│   ├── pid.h               # controlador PID
│   ├── telemetry.h         # struct Telemetry_t, mutex
│   └── ultrasonic.h        # HC-SR04
└── Src/
    ├── app_freertos.c      # init das tarefas
    ├── app_tasks.c         # maquina de estados + PID + calibracao
    ├── bluetooth.c         # parser $GET/$SET/$CMD
    ├── encoder.c           # leitura de pulsos
    ├── lcd_i2c.c           # driver LCD
    ├── line_sensor.c       # leitura ADC + min/max + centroide
    ├── main.c              # init HAL + RTOS
    ├── motor.c             # PWM dos motores
    ├── pid.c               # algoritmo PID
    ├── stm32g4xx_hal_msp.c # init MSP
    ├── stm32g4xx_it.c      # ISRs (encoders, ultrassom, BT)
    ├── system_stm32g4xx.c  # clock
    ├── telemetry.c         # getters/setters com mutex
    └── ultrasonic.c        # trigger HC-SR04
```

## Arquivos nao commitados

Gerados localmente pelo CubeMX / build - ja estao no `.gitignore`:
- `Projeto Final/Drivers/`
- `Projeto Final/Middlewares/`
- `Projeto Final/Debug/`
- `.metadata/`, `.settings/`, `.cproject`, `.project`

Cada pessoa roda **Generate Code** no `.ioc` apos o `git pull`.

---

## Disciplina

ES670 - Projeto Final - Grupo B1
