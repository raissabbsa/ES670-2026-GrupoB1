# Guia de teste do seguidor de linha

Este guia descreve a ordem recomendada para fazer o carrinho seguir a linha e como interpretar os dados de debug.

## 1. Defaults no firmware (a partir de 01/jul/2026)

Os valores abaixo ja estao gravados em `app_config` e `motor.c`. Nao eh
obrigatorio enviar `$SET` antes de `$CMD,START`. O Bluetooth serve
apenas para ajuste fino sem recompilar.

```text
KP  = 0.25
KI  = 0.00
KD  = 0.02
SPD = 0.25
LSCL = 1.00
RSCL = 1.00
```

Fluxo minimo: gravar firmware, ligar, enviar `$CMD,START`. Sem nenhum
comando previo.

## 2. Problema corrigido no firmware

Se o carrinho parado imprime `Cruzamento!` logo apos iniciar, isso nao e ajuste de `KP/KI/KD`.

O sintoma aparece quando os sensores tem leituras quase uniformes, mas o firmware conta muitos sensores ativos:

```text
IR[193 210 187 214 219] N[89 87 90 87 86] spr=32 act=5
```

Mesmo com `N` alto, esses valores nao tem contraste suficiente entre sensores. O firmware foi ajustado para contar sensor ativo somente quando houver contraste relativo, evitando falso cruzamento parado.

## 3. Por que o robo saia da linha em reta (versao anterior)

A versao anterior da calibracao (`STATE_CALIBRATING`) mantinha o robo
parado 650 ms sobre a fita. O range min/max coletado por sensor era
pequeno (~30-50 ADC units), as vezes abaixo do minimo aceito (40).
Quando a calibracao era rejeitada, o firmware voltava para os defaults
genericos (120/900), que nao correspondem a escala real do hardware
(~80-300). Normalizando as leituras com esses defaults, o valor ficava
em torno de 0.10 - abaixo do threshold minimo de 0.25. Resultado:

- `CountNormalizedActive` retornava 0
- `LineSensor_GetState` classificava como `LINE_LOST`
- O firmware entrava em modo BUSCA -> roda direita mais rapida -> vira
  pra esquerda -> sai da pista em reta

Correcoes aplicadas:

- `STATE_CALIBRATING` agora varre 350 ms girando devagar + 600 ms parado
  sobre a fita (total 950 ms). Garante range minimo de calibracao.
- Defaults de min/max sao por sensor (5 valores), nao genericos.
- Novo estado `LINE_ON_TRACK_LOW_CONTRAST` para casos de baixo
  contraste (nao cai em LOST se ha spread bruto).
- `CountNormalizedActive` aceita o pico como 1 sensor ativo quando ha
  contraste medio mas nenhum sensor cruza o threshold absoluto.
- Busca inteligente usa o pico normalizado (nao o `last_valid_error`)
  para decidir a direcao quando a calibracao eh valida.

## 4. Comandos Bluetooth

Envie comandos terminados por Enter/CR/LF.

```text
$CMD,START        inicia alinhamento, calibracao e seguimento
$CMD,STOP         para e volta para idle
$CMD,FWD          modo manual para frente
$CMD,LEFT         modo manual esquerda
$CMD,RIGHT        modo manual direita
$GET,PID          retorna KP, KI, KD e SPD ativos
$GET,MOT          retorna escala dos motores esquerdo/direito
$GET,MON          retorna telemetria
$SET,KP,0.20      ajusta proporcional
$SET,KI,0.00      ajusta integral
$SET,KD,0.02      ajusta derivativo
$SET,SPD,0.25     ajusta velocidade base
$SET,LSCL,1.00    escala do motor esquerdo
$SET,RSCL,1.00    escala do motor direito
$SET,MON,1        liga telemetria automatica
$SET,MON,0        desliga telemetria automatica
```

Respostas esperadas:

```text
$OK
$ERR
$PID,0.250,0.000,0.020,0.250
$MOT,1.000,1.000
```

`$GET,PID` e `$GET,MOT` sao **diagnostico** - confirmam o que o
firmware enxerga. Nao sao setup obrigatorio. Use-os antes de
`$CMD,START` se quiser validar os valores que estao ativos.

## 5. Interpretacao do debug dos sensores

Exemplo:

```text
IR[191 207 185 176 217] N[89 87 90 91 86] spr=41 err=-30 cen=19 ctr=26 act=5 cal=1 m2=120/900
```

Campos importantes:

- `IR[...]`: leituras brutas dos 5 sensores.
- `N[...]`: leituras normalizadas em escala `00..99`.
- `spr`: diferenca entre maior e menor leitura bruta.
- `err`: erro de linha em centesimos. `0` e centralizado.
- `cen`: centro detectado em decimos. `20` significa sensor central ideal `2.0`.
- `ctr`: centro alvo em decimos. Exemplo: `26` significa alvo `2.6`.
- `act`: quantidade de sensores considerados ativos.
- `cal`: `1` indica calibracao valida.
- `m2`: minimo/maximo calibrado do sensor central.

Valores esperados:

- Linha central: `err` perto de `0`, `act` entre `1` e `3`.
- Fora da linha: `act=0`.
- Curva: `err` deve mudar de sinal conforme a linha vai para esquerda/direita.
- Cruzamento real: `act>=4` com contraste bruto suficiente.

Se `act=5` com todos os `N` quase iguais, ainda ha falso cruzamento.

Diagnostico rapido:

- `act=0` e `m2=80/320` (ou outro valor nao-default): calibracao ok mas
  thresholds nao batem. Possivel polaridade errada.
- `act=0` e `m2=80/320` mas o robo segue reto: estado
  `LINE_ON_TRACK_LOW_CONTRAST` esta sendo aplicado (correto).
- `act>=1` parado e `BUSCA` no log: calibracao ou estado quebrado. Ver
  `cal=1` e `spr`.

## 6. Ordem obrigatoria de teste

### Passo 1: teste parado em debug

Antes de tentar seguir pista inteira, coloque o carrinho parado e confira:

1. Fora da linha: `act=0`.
2. Linha no centro: `err` perto de `0`, `act >= 1`.
3. Linha na esquerda: `err` deve mudar para um lado, `act >= 1`.
4. Linha na direita: `err` deve mudar para o lado oposto, `act >= 1`.

Se o sinal do erro estiver invertido, nao ajuste PID. Corrija o sinal da aplicacao do motor.

### Passo 2: calibrar motores

Use modo manual:

```text
$SET,LSCL,1.00
$SET,RSCL,1.00
$CMD,FWD
```

Se puxar para esquerda, aumente o motor esquerdo ou reduza o direito:

```text
$SET,LSCL,1.05
```

Se puxar para direita:

```text
$SET,RSCL,1.05
```

Ajuste em passos de `0.03` a `0.05`. Faixa util: `0.85..1.20`.

### Passo 3: primeiro teste de PID

Os defaults do firmware ja sao `KP=0.25 / KD=0.02 / SPD=0.25`. Para
comecar, basta enviar `$CMD,START`. Se quiser ajustar, envie os
`$SET` antes:

```text
$SET,SPD,0.25
$SET,KP,0.25
$SET,KI,0.00
$SET,KD,0.02
$CMD,START
```

Faixas iniciais:

- `SPD`: `0.20..0.35`
- `KP`: `0.15..0.45`
- `KI`: manter `0.00` no inicio
- `KD`: `0.00..0.08`

### Passo 4: ajuste fino

Se sai da linha sem reagir:

```text
$SET,KP,0.30
```

Depois aumente em passos de `0.05`.

Se oscila muito:

```text
$SET,KD,0.04
```

Depois teste `0.06`, `0.08`.

Se continua oscilando, reduza `KP`.

Nao use `KI` ate o carrinho completar voltas estaveis. Quando usar, comece em:

```text
$SET,KI,0.001
```

Faixa maxima inicial recomendada: `0.001..0.010`.

## 7. Ordem de diagnostico quando falhar

1. `Cruzamento!` parado: problema de deteccao de sensor/cruzamento, nao PID.
2. `act=0` em cima da linha: possivelmente calibracao nao convergiu.
   Olhe `m2` no debug. Se for `80/320` (ou o valor que voce ajustou
   como default), a calibracao foi aceita. Se for o valor antigo
   `120/900`, o firmware foi regredido.
3. `act=0` com `spr > 8`: estado `LINE_ON_TRACK_LOW_CONTRAST` foi
   aplicado. Se o robo segue reto mas sem correcao, o PID esta com
   KP=0 ou entrada de erro zerada.
4. `err` com sinal invertido: aplicacao de motor ou sinal do erro invertido.
5. Manual `FWD` puxa para lado: ajuste `LSCL/RSCL`.
6. Segue reta mas perde curva: aumente `KP` ou reduza `SPD`.
7. Oscila em reta: reduza `KP` ou aumente `KD`.

## 8. Configuracao base

Os valores abaixo ja estao ativos por padrao. Sobrescreva via Bluetooth
apenas se precisar ajustar:

```text
$SET,SPD,0.25
$SET,KP,0.25
$SET,KI,0.00
$SET,KD,0.02
$SET,LSCL,1.00
$SET,RSCL,1.00
$GET,PID     # opcional: confirma os valores ativos
$GET,MOT     # opcional: confirma escala dos motores
$CMD,START
```

Suba velocidade somente depois de seguir a linha em baixa velocidade.

