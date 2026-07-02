# Guia de Teste - Circuito Completo (sem Bluetooth)

> **Para:** Ricardo
> **Objetivo:** calibrar e fazer o robo seguir o circuito completo, ajustando parametros pelos botoes fisicos

---

## 1. Mapa de botoes

A placa tem 5 botoes. Cada um faz uma coisa:

```
   +-------------------+
   |  [Cima]    [Dir]  |  <- Cima = Enter/Start
   |  [Esq]    [Baixo] |     Baixo = Debug
   +-------------------+     Esq = Calibra Sensor
   |  [SWITCH FRONTAL] |     Dir  = Calibra Motor
   +-------------------+     Frontal = Panico
```

| Botao | Pino | Funcao | Quando usar |
|-------|------|--------|-------------|
| **Cima (Enter)** | PA5 | Inicia o seguimento OU finaliza calibracao de sensor | Inicio do teste OU para de mexer o robo na calib S |
| **Baixo** | PA7 | Liga/desliga modo DEBUG dos sensores | Para ver o que o robo esta vendo |
| **Esquerdo** | PC8 | Inicia calibracao de SENSOR (passiva) | Antes de seguir a pista, para capturar min/max |
| **Direito** | PC9 | Inicia calibracao de MOTOR (ativa) | Para calibrar LSCL/RSCL (rodas andam retas) |
| **Frontal** | PD2 | PANICO - para o robo IMEDIATAMENTE | Emergencia |

**Sinalizacao do LED amarelo (LED_Y):**
- **Apagado**: IDLE / parado, esperando comando
- **Aceso**: modo DEBUG ligado
- **Piscando rapido** (5 Hz): calibracao em andamento (sensor ou motor)
- **Aceso durante seguimento**: tudo normal

---

## 2. O que cada parametro faz

### KP (Proporcional) - "Quao forte o robo reage ao erro"

- **O que faz**: quando o robo ve a linha fora do centro, gira com forca proporcional ao erro
- **Se KP muito alto**: o robo oscila (zigue-zague) em reta
- **Se KP muito baixo**: o robo nao reage a curvas, sai da pista
- **Faixa util**: 0.20 a 0.50
- **Default**: 0.35

### KI (Integral) - "Corrige erro acumulado"

- **O que faz**: acumula erros ao longo do tempo, compensa desvios constantes
- **Recomendacao**: MANTER EM 0. NAO MEXER.
- **Quando mexer**: so se o robo consistentemente fica de um lado (drift)
- **Se KI > 0**: introduz overshoot, instabilidade
- **Faixa**: 0.000 a 0.010 (comecar com 0.001 se precisar)

### KD (Derivativo) - "Amortece oscilacao"

- **O que faz**: reage a mudancas rapidas, amortece o PID
- **Se KD muito alto**: o robo fica "pesado", reage devagar
- **Se KD muito baixo**: nao amortece, robo oscila
- **Faixa util**: 0.00 a 0.10
- **Default**: 0.04

### SPD (Velocidade base)

- **O que faz**: velocidade media do robo. 0.0 = parado, 1.0 = maximo
- **Se SPD muito alto**: o robo pode nao reagir a tempo em curvas
- **Se SPD muito baixo**: robo lento demais
- **Faixa util**: 0.20 a 0.40
- **Default**: 0.30

### LSCL (Escala motor Esquerdo)

- **O que faz**: multiplica a potencia enviada ao motor esquerdo
- **LSCL < 1.0**: motor esquerdo recebe MENOS potencia (robo puxa para esquerda)
- **LSCL > 1.0**: motor esquerdo recebe MAIS potencia
- **Faixa**: 0.50 a 1.50
- **Default**: 0.95

### RSCL (Escala motor Direito)

- Mesmo conceito, para o motor direito
- **Default**: 1.20

### Parametros que NAO dao para mudar pelos botoes

- **KI**: fixo em 0 (nao da para mudar sem Bluetooth)
- **Centro alvo (CTR)**: calculado na calibracao
- **KP, KD, SPD, LSCL, RSCL**: dao para mudar via Bluetooth (mas voce nao tem)

**IMPORTANTE:** Sem Bluetooth, voce NAO pode mudar esses parametros em tempo real. Eles estao gravados no firmware. Se quiser mudar, precisa regravar o firmware.

---

## 3. Valores default (gravados no firmware)

Estes sao os valores que estao gravados AGORA. Se nao funcionarem bem, va precisar regravar com outros valores.

| Parametro | Valor |
|-----------|-------|
| KP | 0.35 |
| KI | 0.00 |
| KD | 0.04 |
| SPD | 0.30 |
| LSCL | 0.95 |
| RSCL | 1.20 |
| min sensor | 300 |
| max sensor | 1100 |

---

## 4. Procedimento passo a passo

### Passo 1: Calibrar Sensor (Botao Esquerdo)

**O que faz:** o firmware captura os valores min/max dos 5 sensores IR enquanto voce move o robo manualmente sobre a pista.

**Como fazer:**

1. Coloque o robo **perto da fita** (em cima ou ao lado, nao importa)
2. Aperte o botao **ESQUERDO** uma vez
3. LED comeca a **piscar rapido**
4. **Mova o robo manualmente** sobre a pista:
   - Passe a fita sob todos os 5 sensores
   - Mexa o robo em pelo menos 3 posicoes diferentes (chao preto, fita, chao preto do outro lado)
   - Continue mexendo por 10-20 segundos
5. Aperte o botao **CIMA** para finalizar a calibracao
6. LED para de piscar, fica aceso
7. O robo imprime no LCD: `Calib S OK (Enter): pol=LIGHT mean=XXX ...`

**O que significa o log:**
- `pol=LIGHT`: linha branca em chao preto (correto)
- `pol=DARK`: linha preta em chao claro (pista errada)
- `mean=XXX`: media das leituras. Tipicamente 500-900
- `m=[...]/[...]`: min e max por sensor. Se max for muito proximo de min, calib ruim

**Se a polaridade estiver errada** (DARK quando deveria ser LIGHT), a calibracao nao funciona. Nesse caso, nao da para corrigir sem Bluetooth. Use o firmware com a polaridade correta (ou troque o sensor/luz do laboratorio).

### Passo 2: Calibrar Motores (Botao Direito)

**O que faz:** o robo anda sozinho por 3 segundos e calcula LSCL/RSCL baseado em quanto cada roda efetivamente girou.

**Pre-requisito:** superficie **plana**, com pelo menos **50cm de espaco livre** a frente. Os encoders devem estar funcionando (gire as rodas manualmente antes e veja se o contador muda no LCD debug).

**Como fazer:**

1. Coloque o robo em superficie **plana** (pode ser em cima da fita ou fora, nao importa)
2. Aperte o botao **DIREITO** uma vez
3. LED pisca rapido
4. O robo:
   - Anda reto para frente por 2 segundos
   - Para 200ms
   - Anda reto para tras por 1 segundo
   - Para 200ms
5. O robo imprime no LCD: `Calib M OK: L=XXX R=XXX ratio=X.XX LSCL=X.XX RSCL=X.XX`

**O que significa o log:**
- `L=XXX` pulsos da roda esquerda
- `R=XXX` pulsos da roda direita
- `ratio=L/R`: se for 1.0, ambas rodas andaram igual. Se for 0.85, esquerda andou 85% do que a direita
- `LSCL=0.85`: motor esquerdo vai receber 85% da potencia (compensacao)
- `RSCL=1.00`: motor direito fica normal

**Se a calibracao falhar** (log `Calib M FAIL: encoders nao geraram pulsos`):
- Os encoders nao estao conectados ou funcionando
- Verifique o cabo dos encoders
- Tente `$SET,LCDDBG,1` (mas voce nao tem BT)
- Nesse caso, use os valores default LSCL=0.95, RSCL=1.20 (que estao no firmware)

### Passo 3: Iniciar Seguimento (Botao Cima/Enter)

**O que faz:** o robo alinha sobre a fita, faz uma calibracao curta (1.2s) e comeca a seguir.

**Como fazer:**

1. Coloque o robo **em cima da fita** (a fita deve estar sob os sensores centrais, sensores 2-3-4)
2. Aperte o botao **CIMA**
3. O robo:
   - Tenta centralizar sobre a fita (pode dar "Centralizado timeout", e normal)
   - Faz calibracao curta de 1.2 segundos
   - Comeca a seguir a linha
4. Log no LCD: `=== SEGUINDO LINHA ===`
5. O robo segue a linha. Se sair, voce precisa regravar o firmware com outros parametros.

### Passo 4: Parar (Botao Frontal ou Desligar)

**Para parar de emergencia:** aperte o botao **FRONTAL**. O robo para IMEDIATAMENTE com freio.

**Para retomar:** desligue e ligue a placa OU aperte **Cima** de novo.

---

## 5. Modo DEBUG (Botao Baixo)

**O que faz:** mostra no LCD o que os sensores estao vendo em tempo real. Util para diagnosticar.

**Como usar:**

1. Ligue a placa
2. Aperte **BAIXO** uma vez
3. LED acende
4. LCD mostra: `IR[AAA BBB CCC DDD EEE] N[nn nn nn nn nn] spr=XX err=YY cen=ZZ ctr=WW act=N cal=X m2=...`
   - `IR[...]`: leituras brutas (0-4095)
   - `N[...]`: leituras normalizadas (0-99)
   - `spr`: contraste entre maior e menor leitura
   - `err`: erro de linha (0 = centralizado, ±30 = linha fora)
   - `cen`: centro detectado (20 = meio, 0 = esquerda, 40 = direita)
   - `act`: numero de sensores ativos
5. Aperte **BAIXO** de novo para sair

**O que observar:**

- **Em cima da fita**: `err` perto de 0, `act >= 1`, `cen` ~ 20
- **Fora da fita (chao preto)**: `err` varia, `act = 0`, `spr` cai
- **Curva para esquerda**: `err` fica negativo, `cen` < 20
- **Curva para direita**: `err` fica positivo, `cen` > 20

**Se `err` nao muda de sinal quando voce move o robo lateralmente**: o sensor de calibracao esta quebrado. Me avise.

---

## 6. Cruzamento e Fim de Percurso

**Comportamento esperado na pista:**

- **Em curva**: o robo segue normalmente
- **Em cruzamento** (1o perpendicular no caminho): o robo **passa reto** por 400ms e volta a seguir
- **Em fim de percurso** (ultimo perpendicular): o robo **para** com buzzer

**Log esperado:**
```
Cruzamento (evento 1)        <- primeiro perpendicular, passa reto
=== FIM DE PERCURSO ===       <- segundo perpendicular, para
```

Se o robo para no cruzamento OU nao para no fim, ha um problema de calibracao de sensor.

---

## 7. Botao de Panico

**Botao FRONTAL** = panico. Aperte se:
- O robo esta saindo da pista rapido
- O robo bateu em algo
- A linha de chegada nao funciona e o robo continua

**O que acontece:**
- Motores em curto (freio)
- Buzzer toca
- Log: `!!! PANICO !!!`
- Estado: parado

**Para retomar:** desligue/ligue a placa.

---

## 8. Diagnostico de Problemas

### Problema: o robo oscila em reta (zigue-zague)

**Causa:** KP muito alto OU KD muito baixo
**Solucao:** precisa regravar com KP menor ou KD maior

**Valores sugeridos para teste:**
- KP=0.25, KD=0.06
- KP=0.30, KD=0.05
- KP=0.35, KD=0.04 (default)

### Problema: o robo sai nas curvas

**Causa:** KP muito baixo OU velocidade muito alta
**Solucao:** precisa regravar com KP maior ou SPD menor

**Valores sugeridos:**
- KP=0.40, KD=0.04, SPD=0.25
- KP=0.45, KD=0.04, SPD=0.20
- KP=0.50, KD=0.06, SPD=0.20

### Problema: o robo para no meio da pista

**Causa provavel:** a busca (BUSCA) nao funciona bem, ou a calibracao de sensor falhou
**Solucao:** verifique o debug. Se `err` varia corretamente mas o robo nao anda, problema de PID. Se `err=0` sempre, problema de sensor.

### Problema: o robo anda em circulo

**Causa:** LSCL/RSCL errados. O robo esta com compensacao de motor muito agressiva
**Solucao:** refaca a calibracao de motor (`$CMD,CALIB,M` no BT, ou botao Direito). Se nao funcionar, regravar com LSCL=1.0, RSCL=1.0.

### Problema: o robo nao sai da inércia (fica parado)

**Causa:** SPD muito baixo OU atrito do tatami muito alto
**Solucao:** aumente SPD ou escale as rodas (LSCL/RSCL maiores)

### Problema: 1o cruzamento para o robo (deveria passar)

**Causa:** a fita do cruzamento eh larga demais, parecendo fita de chegada
**Solucao:** sem Bluetooth nao tem o que fazer. Regravar com a fita sendo mais estreita.

---

## 9. Fluxo resumido de teste (cola)

```
1. Ligar a placa
2. Posicionar robo perto da fita
3. Botao ESQUERDO  -> LED pisca
4. Mover robo sobre a pista (10-20s)
5. Botao CIMA      -> finaliza calib sensor (LED acende)
6. Posicionar robo em superficie plana
7. Botao DIREITO   -> robo anda 3s sozinho
8. Posicionar robo sobre a fita
9. Botao CIMA      -> inicia seguimento
10. Robo segue a linha
11. Botao FRONTAL  -> para (emergencia) OU esperar fim de pista
```

**Se der problema:** anote o que aconteceu (oscilou, saiu na curva, parou, etc.) e me passe para eu ajustar o firmware.

---

## 10. O que anotar se der problema

Quando o teste falhar, anote:

1. **Onde o robo falhou**: reta, curva, cruzamento, fim de pista?
2. **O que aconteceu**: oscilou, saiu, parou, virou errado?
3. **Log do LPUART1 (115200)**: ultimas linhas antes do problema
4. **Botoes usados**: em que ordem apertou
5. **Pista**: tatami? chao liso? tem rampa?

Essas informacoes me ajudam a ajustar o firmware para a proxima versao.

---

## 11. Mudancas que NAO dao para testar (precisam de Bluetooth)

- Trocar KP/KD/SPD em tempo real
- Trocar LSCL/RSCL depois de calibrar
- Forcar polaridade (DARK/LIGHT)
- Ver telemetria ($MON)
- Modo debug do LCD com contadores de motor/encoder/bateria
- Controle manual F/B/L/R (car controller)

Se voce quiser testar essas coisas, precisa do modulo Bluetooth. Caso contrario, so da para testar o fluxo completo: calibrar e seguir.

---

## 12. Resumo dos valores de teste

Aqui estao os valores que VAO funcionar melhor (em ordem de prioridade para tentar):

| Parametro | Conservador | Balanceado | Agressivo |
|-----------|-------------|------------|-----------|
| KP | 0.25 | 0.35 | 0.45 |
| KI | 0.00 | 0.00 | 0.00 |
| KD | 0.06 | 0.04 | 0.02 |
| SPD | 0.25 | 0.30 | 0.20 |
| LSCL | 1.00 | 0.95 | 0.95 |
| RSCL | 1.00 | 1.20 | 1.20 |

**Interpretacao:**
- **Conservador**: anda devagar, nao oscila, mas nao faz curvas fechadas
- **Balanceado** (default): compromisso entre velocidade e precisao
- **Agressivo**: rapido, faz curvas fechadas, pode oscilar em reta
