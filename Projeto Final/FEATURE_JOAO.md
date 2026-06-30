# feature/joao — Extensões sobre a `main`

> Branch criada a partir da `main` para adicionar funcionalidades de **safety**,
> **UX** e **telemetria** sem mudar a arquitetura existente (FSM
> `ALIGNING → CALIBRATING → FOLLOWING` da Raissa, drivers `motor`/`pid`/
> `line_sensor`/`encoder`).

## O que esta branch adiciona

| Camada    | Arquivos novos                                | Função                                               |
|-----------|-----------------------------------------------|------------------------------------------------------|
| Drivers   | `ultrasonic.[ch]`                             | HC-SR04 via TIM20_CH1 (trigger) + TIM3_CH1 (echo DMA)|
| Drivers   | `bumper.[ch]`                                 | Switch frontal PD2 — latch de emergência             |
| Drivers   | `bluetooth.[ch]`                              | USART3/HC-05 ringbuffer RX + parser ASCII            |
| Drivers   | `odometry.[ch]`                               | (x, y, θ) integrado dos encoders                     |
| Drivers   | `lcd_hd44780_i2c.[ch]`                        | LCD 16x2 sobre I2C2 (PCF8574 @ 0x27)                 |
| Tasks RTOS| `task_safety.c`                               | 50Hz — checa obstáculo + bumper, integra odometria   |
| Tasks RTOS| `task_display.c`                              | 2Hz — atualiza LCD com estado, vel, dist, obstáculo  |
| Tasks RTOS| `task_telemetry.c`                            | 5Hz — CSV via LPUART + Bluetooth, drena BT RX        |

E adiciona à FSM existente em `app_tasks.h`:

```c
typedef enum {
    STATE_IDLE, STATE_ALIGNING, STATE_CALIBRATING, STATE_FOLLOWING,
    STATE_IN_CROSSING, STATE_STOPPING, STATE_STOPPED, STATE_DEBUG,
    /* NOVO: */
    STATE_STOP_OBSTACLE,   /* parado por obstáculo (retoma sozinho)        */
    STATE_EMERGENCY,       /* bumper acionado (só sai por reset/STOP via BT)*/
} LineFollower_State;
```

## Tasks (visão geral)

```
prio High       SafetyTask       50Hz   ── ultrasonic + bumper + odometry
prio High-1     MotorCtrlTask   100Hz   ── (já existia, Raissa)
prio Normal     LineCtrlTask     50Hz   ── (já existia, Raissa)
prio Low        TelemetryTask     5Hz   ── CSV via UART/BT
prio Low        DisplayTask       2Hz   ── LCD
```

## Comandos Bluetooth (HC-05 @ 9600 8N1)

Enviar linhas terminadas em `\n`. Respostas: `OK`, `ERR`, `PONG`.

| Comando         | Efeito                                         |
|-----------------|------------------------------------------------|
| `PING`          | sanity-check                                   |
| `GO`            | inicia (equivale ao botão ENTER)               |
| `STOP`          | para tudo                                      |
| `STATE?`        | devolve `STATE:<num>`                          |
| `VEL=<float>`   | ajusta `app_config.base_speed`                 |
| `KP_L=<float>`  | ajusta `app_config.line_Kp` (PID seguidor)     |
| `KI_L=<float>`  | ajusta `app_config.line_Ki`                    |
| `KD_L=<float>`  | ajusta `app_config.line_Kd`                    |
| `KP_V=<float>`  | ajusta `app_config.speed_Kp`                   |
| `KI_V=<float>`  | ajusta `app_config.speed_Ki`                   |
| `KD_V=<float>`  | ajusta `app_config.speed_Kd`                   |

**Telemetria CSV** sai a 5Hz pela LPUART e pelo HC-05:
```
<state>,<base_speed>,<dist_cm>,<x>,<y>,<theta_deg>,<obstacle_cm>
```

## Pendências de CubeMX (para o `.ioc`)

A maioria do `.ioc` já está pronta na `main`. Faltam **2 ajustes** para os
novos drivers funcionarem 100%:

1. **I2C2 → DMA TX**
   - Em `I2C2 → DMA Settings`, adicione `I2C2_TX` em modo `Normal` (não
     circular), Data Width = Byte.
   - Sem isso, o LCD vai falhar nas chamadas a `HAL_I2C_Master_Transmit_DMA`.
   - **Workaround temporário**: trocar `HAL_I2C_Master_Transmit_DMA` por
     `HAL_I2C_Master_Transmit` (blocking) em `lcd_hd44780_i2c.c` — mas o
     ideal é configurar o DMA.

2. **TIM3 → DMA do canal de Input Capture (echo do HC-SR04)**
   - Em `TIM3 → DMA Settings`, adicione `TIM3_CH1` em modo `Circular`,
     Data Width = Half Word, mem-inc = enabled.
   - Sem isso, `HAL_TIM_IC_Start_DMA` falha.

Depois de aplicar, **regenerar código** no CubeMX preservando os USER CODE.

## Fluxo da FSM (após esta branch)

```
                          STATE_EMERGENCY  ◀── bumper (latch)
                                ▲                        ▲
                                │                        │
   IDLE ─ENTER─▶ ALIGNING ─▶ CALIBRATING ─▶ FOLLOWING ───┤
    ▲                                          ▲   │     │
    │                                          │   ▼     │
    └─── (BT STOP) ◀──── STOPPING ◀────── IN_CROSSING    │
                                              │          │
                                              ▼          │
                                       STATE_STOP_OBSTACLE◀──
                                              │              ultrassônico
                                              └──> obstáculo livre > 18cm
                                                   volta a FOLLOWING
```

## Como compilar/flashar

Mesmo procedimento da `main`: importar no STM32CubeIDE, regenerar o `.ioc`
após aplicar os 2 ajustes acima, `Project → Build All`, `Run As → STM32
C/C++ Application`.

Autor das extensões: **João Santos** — sessão 29/06/2026.
