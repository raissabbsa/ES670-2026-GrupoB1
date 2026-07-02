# Checklist de Testes - Projeto Final (jul/2026)

> **Status:** todas as alteracoes abaixo estao em `git status` (nao commitadas).
> Antes de testar: gravar o firmware novo na placa.

---

## Indice

1. [Pre-requisitos: gravar o firmware](#1-pre-requisitos)
2. [Teste 0: confirmar boot e BT](#2-teste-0-boot-e-bt)
3. [Teste 1: comandos $GET (com echo RAW)](#3-teste-1-comandos-get)
4. [Teste 2: LCD debug mode (encoders, motor, bateria)](#4-teste-2-lcd-debug)
5. [Teste 3: calibracao automatica de sensores (passiva)](#5-teste-3-calib-sensores)
6. [Teste 4: calibracao automatica de motores (ativa)](#6-teste-4-calib-motores)
7. [Teste 5: seguimento da linha (KP/KD/SPD)](#7-teste-5-seguimento)
8. [Teste 6: cruzamento e fim de percurso (contador)](#8-teste-6-cruzamento)
9. [Teste 7: botao de panico](#9-teste-7-panico)
10. [Teste 8: obstaculo (ultrassom)](#10-teste-8-obstaculo)
11. [Teste 9: controle manual via BT (car controller)](#11-teste-9-manual)
12. [Teste 10: telemetria via BT ($MON)](#12-teste-10-telemetria)
13. [Teste 11: LCD modo normal (D, V, H, Bat)](#13-teste-11-lcd-normal)
14. [Criterio de pronto / pontos por item](#14-criterio)

---

## 1. Pre-requisitos

### 1.1 Gravar o firmware

1. Abra STM32CubeIDE.
2. **File -> Open Projects from File System** -> pasta `Projeto Final`.
3. **Project -> Clean** -> **Project -> Build All** (Ctrl+B).
4. **Run -> Debug** (F11) ou **Run -> Run** (Ctrl+F11) para gravar.
5. Espere a placa reiniciar.

### 1.2 Conectar seriais

Voce precisa de **dois terminais seriais** (ou um com duas sessoes):

| Serial | Baud | Usar para |
|--------|------|-----------|
| **LPUART1** | 115200 | Log de debug, calibracao, telemetria, mensagens de estado |
| **USART3** (Bluetooth HC-05) | 9600 | Comandos do celular |

No Linux: `minicom -D /dev/ttyACM0 -b 115200` para LPUART1, ou `cutecom` / `putty` com 2 sessoes.

No Android (celular): instale **Serial Bluetooth Terminal** (ou Arduino Bluetooth Control) e conecte ao HC-05 (PIN 1234 ou 0000).

### 1.3 Verificar que gravou

1. Ligue a placa.
2. No **LPUART1 (115200)**, deve aparecer:
   ```
   Robo OK - serial 115200
   Enter=inicia (alinha sozinho)
   ```

Se nao aparecer, verifique a porta serial e a velocidade.

### 1.4 Atualizar o Bluetooth

No **app do celular (Serial Bluetooth Terminal)**:

1. Conecte ao HC-05.
2. Velocidade: 9600.
3. Certifique que o "newline" esta configurado como **CR+LF** ou **CR** (alguns terminais adicionam LF, outros CR).

---

## 2. Teste 0: Boot e BT

**Objetivo:** confirmar que o Bluetooth esta conectado e respondendo.

### 2.1 Testar $HELP

**Comando (celular):**
```
$HELP
```

**Esperado (LPUART1 ou BT):**
```
$HELP,GET|PID|MOT|MON|ALL,SET|KP|KI|KD|SPD|LSCL|RSCL|LCDDBG|DBGRAW|MON|POL,CMD|START|STOP|FWD|REV|LEFT|RIGHT|JOY|CALIB,S|CALIB,M
```

**Se nao responder:** verifique pareamento Bluetooth, baud (9600), e que o HC-05 esta conectado (LED pisca lento quando pareado).

### 2.2 Testar eco RAW (debug de linha recebida)

**Comando (celular):**
```
$SET,DBGRAW,1
```

**Esperado:** `$OK`

Agora cada comando enviado pelo BT sera ecoado no LPUART1 (serial de debug) com prefixo `>>` para o que foi enviado, e `$RAW[N]="..."` para o que o parser recebeu. Exemplo:

Lado BT (celular):
```
$GET,PID
```

Lado LPUART1 (debug, 115200):
```
$RAW[8]="$GET,PID"
```

Se voce ve `$RAW[N]="<lixo>"`, o terminal do celular esta mandando caracteres extras.

**Para desligar:** `$SET,DBGRAW,0`

---

## 3. Teste 1: Comandos $GET

**Objetivo:** confirmar que os gets respondem corretamente.

### 3.1 $GET,PID

**Comando:**
```
$GET,PID
```

**Esperado:**
```
$PID,0.350,0.000,0.040,0.250
```

KP=0.35, KI=0, KD=0.04, base_speed=0.25.

**Se nao responder:** o problema e a polaridade/acentuacao do terminal. Tente `$GETPID` (sem virgula) tambem.

### 3.2 $GET,MOT

**Comando:**
```
$GET,MOT
```

**Esperado:**
```
$MOT,0.950,1.200
```

LSCL=0.95, RSCL=1.20 (defaults).

### 3.3 $GET,ALL

**Comando:**
```
$GET,ALL
```

**Esperado:**
```
$ALL,KP=0.350,KI=0.000,KD=0.040,SPD=0.250,LSCL=0.950,RSCL=1.200
```

### 3.4 $GET,MON

**Comando:**
```
$GET,MON
```

**Esperado (com robo parado):**
```
$MON,0,0,0,0,0,0
```

distancia, velocidade, heading, bateria, obstaculo, estado.

---

## 4. Teste 2: LCD debug mode

**Objetivo:** confirmar que motor task esta rodando, encoders funcionam, ADC da bateria le algo.

### 4.1 Ativar LCD debug

**Comando:**
```
$SET,LCDDBG,1
```

**Esperado no LCD:**
- Linha 1: `MC:100 E:0` (depois de 1 segundo deve ser `MC:100` ou `MC:200`, etc)
- Linha 2: `Bat:0 R:0` (Bateria raw, R=encoder right)

### 4.2 Verificar motor task

**O que fazer:**
1. Olhe o LCD por 2-3 segundos.
2. Observe o valor de `MC`.

**Esperado:** `MC` cresce ~100x por segundo (100, 200, 300, ...). Se `MC` fica em `0` ou nao muda, **a task do motor NAO esta rodando** - bug grave, me avise.

### 4.3 Verificar encoders

**O que fazer:**
1. Ainda em LCDDBG=1.
2. Com a mao, gire a roda esquerda manualmente (sentido horario ou anti-horario).
3. Olhe o valor de `E` (encoder left) na linha 1.
4. Gire a roda direita e olhe o `R` na linha 2.

**Esperado:** `E` e `R` devem aumentar ou diminuir quando voce gira. Se ficarem sempre `0`, **os encoders NAO estao funcionando** (problema de fiação/TIM16/TIM17/HAL).

**Se encoders nao funcionam:** a calibracao automatica de motor nao vai funcionar. Use `$SET,LSCL,X` / `$SET,RSCL,X` manual.

### 4.4 Verificar bateria

**O que fazer:**
1. Ainda em LCDDBG=1.
2. Olhe o valor de `Bat` na linha 2.

**Esperado:** `Bat` > 100 (tipicamente 600-1200 para bateria 2S LiPo 7.4V via divisor resistivo). Se `Bat = 0`, **o ADC da bateria NAO esta lendo** (problema de hardware no canal ADC).

### 4.5 Desativar LCD debug

**Comando:**
```
$SET,LCDDBG,0
```

LCD volta ao modo normal (D, V, H, Bat).

---

## 5. Teste 3: Calibração automática de sensores (passiva)

**Objetivo:** confirmar que o firmware consegue capturar min/max reais da pista quando o usuario move o robo manualmente.

### 5.1 Preparar ambiente

1. Coloque o robo na pista, perto da fita branca.
2. Abra o terminal LPUART1 (115200) para ver os logs.

### 5.2 Iniciar calibração

**Comando (BT):**
```
$CMD,CALIB,S
```

**Esperado (LPUART1):**
```
Calib S: capturando (mexa o robo)...
```

O LED amarelo (LED_Y) começa a piscar rápido (5 Hz). O celular responde:
```
$Calib S: capturando (mexa o robo, $CMD,CALIB,S,STOP para parar)
```

### 5.3 Mover o robô

1. Mova o robo **manualmente** sobre a pista:
   - Posição A: chão preto longe da fita
   - Posição B: em cima da fita branca
   - Posição C: chão preto do outro lado
   - Repita A-B-C-A-B-C umas 5 vezes em 10 segundos
2. Mexa o robô em várias direções (frente, lado, diagonal).
3. Tente colocar a fita embaixo de cada um dos 5 sensores separadamente.

**O que esta acontecendo:** o firmware le os 5 sensores a cada 20ms e atualiza `s_min[]` e `s_max[]` de cada um.

### 5.4 Parar calibração

**Comando (BT):**
```
$CMD,CALIB,S,STOP
```

OU espere 30 segundos (timeout automatico).

**Esperado (LPUART1):**
```
Calib S OK (stop): pol=LIGHT mean=850 m=[300,310,295,305,300]/[1000,1020,990,1010,1000]
```

**O que verificar no log:**
- `pol=LIGHT` (linha branca em chao preto). Se for `pol=DARK`, o pista tem linha preta - use `$SET,POL,0` para forcar.
- `mean=XXX` deve ser entre 400-1000 (média das leituras). Se for muito fora, o robo viu uma superficie homogenea.
- `m=[...]/[...]` mostra min/max por sensor. O **max deve ser bem maior que o min** (range > 200 indica calibração boa).

**Celular responde:**
```
$Calib S OK (stop)
```

LED para de piscar, fica aceso.

### 5.5 Se o log mostrar tudo zerado

Se `m=[300,...]/[300,...]` (tudo igual), o robo nao foi movido. Repita o teste, mexendo o robo mais.

### 5.6 Se a polaridade estiver errada

Se a pista tem linha branca mas o log mostra `pol=DARK`:
```
$SET,POL,1
```

E repita a calibração.

---

## 6. Teste 4: Calibração automática de motores (ativa)

**Objetivo:** o robo anda sozinho e calcula LSCL/RSCL automaticamente.

### 6.1 Pre-requisito

- **Encoders funcionando** (teste 4.3 deve ter passado)
- Superficie **plana** (sem rampa), com pelo menos 50cm de espaço livre
- Robo **parado** (estado IDLE, sem obstaculo a frente)
- Em qualquer posicao (em cima ou fora da fita, nao importa)

### 6.2 Iniciar calibração

**Comando (BT):**
```
$CMD,CALIB,M
```

**Esperado (LPUART1):**
```
Calib M: andando (~3s)...
Calib M: fwd...
Calib M: rev...
```

**Celular responde:**
```
$Calib M: andando (~3s)...
```

LED pisca rápido. O robo:
1. Anda reto para frente por 2 segundos (potencia 0.30 nos dois motores)
2. Para 200ms
3. Anda reto para tras por 1 segundo
4. Para 200ms
5. Calcula ratio dos encoders

### 6.3 Resultado

**Esperado (LPUART1):**
```
Calib M OK: L=265 R=320 ratio=0.83 LSCL=0.83 RSCL=1.00
```

**O que significa:**
- L=265 pulsos na roda esquerda no total (ida + volta)
- R=320 pulsos na roda direita no total
- ratio = L/R = 0.83 (roda esquerda andou 83% do que a direita andou)
- LSCL=0.83 (esquerdo precisa de menos potencia)
- RSCL=1.00 (direito fica normal)

### 6.4 Possiveis problemas

**Se o robo nao anda:** verifique se a calibracao anterior dos motores (LSCL/RSCL) nao esta zerada. Mande `$GET,MOT` para ver.

**Se a calibracao falha com `Calib M FAIL: encoders nao geraram pulsos`:** os encoders nao estao funcionando. Volte ao Teste 4.3.

**Se o robo anda em circulo (vira muito):** a calibracao calculou um ratio extremo (ex: 0.5). Pode ser que o robo tenha atolado em algo durante o teste. Repita em uma superficie mais limpa.

### 6.5 Verificar que LSCL/RSCL foram aplicados

**Comando (BT):**
```
$GET,MOT
```

**Esperado:** deve mostrar `LSCL` e `RSCL` iguais aos valores do log de calibração.

### 6.6 Ajustar manualmente se necessário

Se a calibração automática nao der resultado bom (robo continua puxando para um lado), ajuste manual:

```
$SET,LSCL,0.85
$SET,RSCL,1.20
```

E teste de novo.

---

## 7. Teste 5: Seguimento da linha (KP/KD/SPD)

**Objetivo:** confirmar que o robo segue a linha em reta e em curva.

### 7.1 Pre-requisito

- Calibração dos sensores feita (Teste 3)
- Calibração dos motores feita (Teste 4)
- Robo em IDLE

### 7.2 Iniciar seguimento

**Coloque o robo sobre a fita** (linha branca no centro dos 5 sensores) e aperte o **botao Enter (Cima)** OU mande:

```
$CMD,START
```

**Esperado (LPUART1):**
```
Ja na fita - centralizando
Centralizado timeout
=== CALIBRANDO (1.2s) ===
=== SEGUINDO LINHA ===
F ON vl=49 vr=50 adj=-11 spr=110
F ON vl=50 vr=50 adj=0 spr=89
...
```

**Nota:** "Centralizado timeout" e normal - significa que o alinhamento nao conseguiu centralizar perfeitamente, mas o firmware prossegue para a calibracao curta (1.2s).

### 7.3 Verificar comportamento

**O que observar:**
- O robo deve seguir a linha sem sair
- `vl` e `vr` devem variar (diferença de velocidade nas curvas)
- `adj` (erro filtrado) deve oscilar em torno de 0
- `spr` deve ser > 30 (bom contraste)

### 7.4 Testar em reta

Coloque o robo no inicio de uma reta. Veja se ele vai reto sem oscilar.

**Se oscilar muito (zigue-zague):** reduza `KP` (ex: `$SET,KP,0.30`)

**Se nao reagir a curva:** aumente `KP` (ex: `$SET,KP,0.40`)

### 7.5 Testar em curva

Coloque o robo no inicio de uma curva. Veja se ele faz a curva sem sair.

**Se sair da curva:** aumente `KD` para amortecer (ex: `$SET,KD,0.06`)

**Se travar na curva:** aumente `KP` (ex: `$SET,KP,0.40`)

### 7.6 Parametros de referencia

Com a calibracao automatica, os valores devem funcionar:

| Parametro | Default | Faixa tipica |
|-----------|---------|--------------|
| KP | 0.35 | 0.20 - 0.50 |
| KI | 0.00 | NAO mexer (manter 0) |
| KD | 0.04 | 0.02 - 0.10 |
| SPD | 0.25 | 0.20 - 0.40 |
| LSCL | 0.95 | 0.80 - 1.20 |
| RSCL | 1.20 | 0.80 - 1.20 |

### 7.7 Parar seguimento

Aperte o **switch frontal** (botao de panico) OU desligue/ligue a placa.

**Log esperado:** `=== FIM DE PERCURSO ===` ou `=== PAROU ===`

---

## 8. Teste 6: Cruzamento e fim de percurso (contador)

**Objetivo:** confirmar que o primeiro cruzamento passa reto e o segundo para.

### 8.1 Cenario

A pista tem 2 eventos perpendiculares à linha de seguimento:
1. **1o evento** = cruzamento (continua reto)
2. **2o evento** = linha de chegada (para)

### 8.2 Iniciar

1. Coloque o robo no inicio da pista.
2. Aperte Enter ou `$CMD,START`.

### 8.3 Observar 1o cruzamento

**Quando o robo passar pelo 1o cruzamento:**

**Esperado (LPUART1):**
```
Cruzamento (evento 1)
```

O robo anda reto por 400ms (atraves da fita perpendicular) e volta a seguir.

**Celular nao envia nada** - o firmware detecta automaticamente.

### 8.4 Observar 2o evento (chegada)

**Quando o robo chegar no fim:**

**Esperado (LPUART1):**
```
Cruzamento (evento 2)
```
OU
```
Fim de percurso (fita larga, evento 2)
=== FIM DE PERCURSO ===
```

O robo para IMEDIATAMENTE, buzzer toca, estado STATE_STOPPING -> STATE_STOPPED.

### 8.5 Possiveis problemas

**Se o robo para no 1o cruzamento:** o evento 1 foi contado como chegada. Causas:
- A 1a fita perpendicular e larga demais (ativa todos os 5 sensores) - nao e um cruzamento
- Calibração ruim - todos os sensores leem normalizado similar mesmo com a fita estreita
- Solução: ajustar thresholds ou fazer a 1a fita mais estreita

**Se o robo continua na chegada:** o 2o evento nao foi detectado. Causas:
- A fita de chegada e estreita demais
- Solução: usar fita mais larga para chegada

**Se o robo nao detecta nenhum:** problema de calibracao. Volte ao Teste 3.

### 8.6 Reiniciar

Desligue/ligue a placa OU faca um reset. O `crossing_count` e resetado quando entra em FOLLOWING de novo.

---

## 9. Teste 7: Botão de pânico

**Objetivo:** confirmar que o switch frontal (PD2) para o robo IMEDIATAMENTE.

### 9.1 Iniciar seguimento normal

1. Calibre e siga a linha.
2. Espere o robo estar em FOLLOWING.

### 9.2 Aperte o botão de pânico

Aperte e segure o **switch frontal** (botao na parte frontal da placa).

**Esperado:**
- O robo para IMEDIATAMENTE (ambos os motores em curto - freio)
- Log: `!!! PANICO !!!`
- Estado: STATE_STOPPING -> STATE_STOPPED

### 9.3 Liberar o botão

O robo continua parado (não retoma). Para retomar:
- Desligar/ligar a placa, OU
- Aperte Enter para reiniciar

---

## 10. Teste 8: Obstáculo (ultrassom)

**Objetivo:** confirmar que o ultrassom detecta obstáculo e para o robo.

### 10.1 Iniciar seguimento

1. Calibre e siga a linha.
2. Espere o robo estar em FOLLOWING.

### 10.2 Colocar obstáculo a frente

Aproximar a mao ou um objeto a **menos de 10cm** do sensor ultrassom (HC-SR04, normalmente na frente da placa).

**Esperado:**
- O robo para em ate 1.5 segundos (debounce de 3 leituras a cada 250ms)
- Log: `!!! OBSTACULO !!!`
- Buzzer toca (som continuo)
- Estado: STATE_STOPPING -> STATE_STOPPED

### 10.3 Remover o obstáculo

O buzzer continua tocando. Para retomar:
- Desligar/ligar a placa, OU
- Aperte Enter

### 10.4 Se nao detectar

**Verifique:**
- O sensor ultrassom esta conectado (trigger em algum pino, echo em outro)
- Os pinos do trigger/echo estao configurados no .ioc
- O sensor esta virado para frente, nao para baixo
- Distancia < 10cm (HC-SR04 tem minimo de 2cm)

---

## 11. Teste 9: Controle manual via BT (car controller)

**Objetivo:** confirmar que o app "Arduino Bluetooth Control" ou similar pode controlar o robo.

### 11.1 Comandos tradicionais ($CMD,XXX)

**Comandos (BT):**
```
$CMD,FWD      # frente
$CMD,REV      # ré
$CMD,LEFT     # esquerda (gira no proprio eixo)
$CMD,RIGHT    # direita
$CMD,STOP     # para
```

**Esperado:** o robo faz a manobra, e o log mostra `=== PAROU ===` para o STOP.

**Para sair do manual:** `$CMD,START` para voltar ao seguimento.

### 11.2 Car controller (1 caractere)

**Configurar o app "Arduino Bluetooth Control":**
- Va em **Car Controller**
- Configure cada botao:
  - Cima: envia `F`
  - Baixo: envia `B`
  - Esquerda: envia `L`
  - Direita: envia `R`

**Teste:**
- Aperte **Cima** no app: robo anda para frente
- Aperte **Baixo**: robo anda para tras
- Aperte **Esquerda**: robo vira para esquerda
- Aperte **Direita**: robo vira para direita
- Solte todos: robo para

**Celular responde:** `$OK` para cada comando.

**Velocidade:** baseada em `app_config.base_speed` (= 0.30). Use `$SET,SPD,0.50` para mais velocidade.

### 11.3 $CMD,JOY (controle direto)

**Comando (BT, com app de terminal):**
```
$CMD,JOY,50,50      # ambas rodas a 50% para frente
$CMD,JOY,0,50       # só direita a 50% (vira esquerda)
$CMD,JOY,50,0       # só esquerda a 50% (vira direita)
$CMD,JOY,-50,-50    # ambas para tras
```

Valores -100 a 100. **Cuidado:** esses valores vao DIRETO para os motores, sem escala LSCL/RSCL.

---

## 12. Teste 10: Telemetria via BT ($MON)

**Objetivo:** confirmar que a telemetria atualiza 1Hz e mostra dados reais.

### 12.1 Ativar monitor automatico

**Comando (BT):**
```
$SET,MON,1
```

**Esperado (BT, a cada 1 segundo):**
```
$MON,0,0,0,0,0,0
$MON,12,5,3,85,400,2
$MON,28,8,5,84,400,2
...
```

Campos: distancia_cm, velocidade_cms, heading_deg, bateria_pct, obstaculo_cm, estado.

### 12.2 Verificar dados

Com o robo em movimento:
- `distancia_cm` deve crescer
- `velocidade_cms` deve ser > 0
- `heading_deg` deve mudar quando o robo vira
- `bateria_pct` deve ser > 0 e < 100

### 12.3 Desativar

```
$SET,MON,0
```

---

## 13. Teste 11: LCD modo normal (D, V, H, Bat)

**Objetivo:** confirmar que o LCD mostra dados 1Hz no modo normal.

**Pre-requisito:** o modo debug deve estar desativado:
```
$SET,LCDDBG,0
```

### 13.1 Iniciar seguimento

1. Calibre e siga a linha.
2. LCD deve mostrar:
   - Linha 1: `D:NNNcm V:NN ` (distancia e velocidade)
   - Linha 2: `H:NNN Bat:NN%` (heading e bateria)

### 13.2 Verificar valores

- `D` deve crescer com o tempo
- `V` deve ser > 0 durante movimento
- `H` deve mudar quando o robo vira
- `Bat` deve estar entre 0-100%

### 13.3 Se LCD mostrar zeros

Confirme que os encoders funcionam (Teste 4.3) e que a bateria le algo (Teste 4.4). Se os dois funcionam, o problema e no caminho entre o motor task e o LCD.

---

## 14. Criterio de pronto

### Resumo dos pontos (10 pontos no total)

| Item | Pontos | Comando/acao para testar |
|------|--------|---------------------------|
| (2,0) Seguir linha | 2,0 | Teste 5, 7 |
| (0,5) Cruzamento | 0,5 | Teste 8 (1o cruzamento) |
| (0,5) Trajeto 1:30 | 0,5 | Teste 5 (deixar 90s) |
| (0,5) Parar no fim | 0,5 | Teste 8 (2o evento) |
| (1,0) Monitor BT 1Hz | 1,0 | Teste 12 |
| (1,0) Config BT | 1,0 | Testes 3, 11 |
| (0,5) Manual BT | 0,5 | Teste 9 |
| (1,0) Seguranca | 1,0 | Testes 7 (panico) + 8 (obstaculo) |
| (1,5) LCD 1Hz | 1,5 | Teste 13 |
| (1,5) FreeRTOS | 1,5 | Validado pela arquitetura |

### Checklist de validacao

- [ ] Boot: LPUART1 mostra "Robo OK"
- [ ] BT: $HELP responde
- [ ] BT: $GET,PID, $GET,MOT, $GET,MON, $GET,ALL respondem
- [ ] LCD: MC cresce 100x/segundo (motor task roda)
- [ ] LCD: encoders E/R mudam quando rodas giram
- [ ] LCD: Bat > 0
- [ ] Calib S: min/max capturados, pol detectado (LIGHT ou DARK)
- [ ] Calib M: LSCL/RSCL calculados e aplicados
- [ ] Seguimento: robo segue reta e curvas
- [ ] 1o cruzamento: robo passa reto
- [ ] 2o evento (chegada): robo para com buzzer
- [ ] Botao panico: para imediato
- [ ] Obstaculo (< 10cm): para com buzzer
- [ ] Manual BT: comandos F/B/L/R funcionam
- [ ] $MON: telemetria atualiza 1Hz
- [ ] LCD normal: D, V, H, Bat atualizam

### O que NAO esta implementado

- **Calibração de 9 eixos (IMU)** - nao tem IMU no projeto
- **Fim de pista por sensor de contato** - botao frontal funciona como pânico
- **Telemetria com postura (x, y) no LCD** - o LCD 16x2 nao tem espaço; use $MON no celular
- **KI** - recomendado manter em 0, nao mexer

### Relatar problemas

Ao reportar problema, inclua:
1. Qual teste falhou
2. O comando enviado
3. A resposta esperada vs a resposta real
4. O log completo do LPUART1 (se disponivel)
5. Valores de LCDDBG (MC, E, R, Bat) se o problema envolver hardware
