# Planmode: portar correcoes de calibracao/aplicacao do ES070-2024 para ES670-2026

## Objetivo

Aplicar no ES670-2026 os aprendizados do ES070-2024 sobre calibracao dos sensores de linha, normalizacao e decisao de movimento, sem substituir a arquitetura atual com FreeRTOS, telemetria, Bluetooth, ultrassom e busca de linha.

## Comparacao relevante

### ES070-2024

- `seguidorLinha/Core/Src/controller.c` usa arrays fixos de calibracao por sensor:
  - `iSensorMax[5] = {8500, 8500, 8500, 8500, 11200}`
  - `iSensorMin[5] = {4500, 5000, 4900, 4700, 3600}`
- A funcao `normalizarValor()` limita a leitura entre minimo/maximo e converte para `0.0..1.0`, com `1.0` representando preto.
- A atualizacao dinamica de min/max foi comentada no seguidor por causa de outliers, deixando a calibracao estavel durante a corrida.
- A aplicacao usa limiares normalizados simples:
  - centro alto: `0.55`
  - laterais altas: `0.25`
- Ha contadores de persistencia para fora de pista e fim de percurso.

### ES670-2026

- `Core/Src/line_sensor.c` ja tem leitura por sensor, filtro de outliers, centro estimado, polaridade e erro interpolado.
- A calibracao atual comeca em `LineSensor_ResetCalibration()` com min/max extremos e depende de `LineSensor_UpdateCalibration()`.
- No estado `STATE_CALIBRATING`, a task coleta centro por cerca de 650 ms, mas nao atualiza min/max nesse trecho; a calibracao de faixa depende mais do alinhamento anterior.
- O controle em `STATE_FOLLOWING` usa `spread >= LINE_LOST_SPREAD_MIN`, erro interpolado filtrado e PID.
- O campo `sensor_threshold` em `AppConfig_t` existe, mas nao participa da decisao.

## Correcoes a implementar

1. Separar calibracao de faixa e calibracao de centro
   - Criar estrutura `LineSensor_Calibration` com `min[5]`, `max[5]`, `polarity`, `center_target` e flag `valid`.
   - Manter valores default seguros inspirados no ES070, mas convertidos para a escala real do ES670, que aparenta trabalhar com leituras abaixo de `LINE_ADC_MAX_VALID = 2000`.
   - Nao depender de min/max extremos durante a corrida.

2. Tornar a normalizacao o caminho principal
   - Expor uma API `LineSensor_GetNormalizedAll(raw, norm)`.
   - Usar `LineSensor_Normalize()` para calcular contagem ativa, centro e erro, evitando misturar heuristicas de raw spread com faixa calibrada.
   - Preservar fallback por `spread` somente quando a calibracao ainda nao for valida.

3. Corrigir a rotina de calibracao do ES670
   - Ao entrar em `STATE_CALIBRATING`, chamar `LineSensor_ResetCalibration()`.
   - Durante os 650 ms, chamar `LineSensor_UpdateCalibration(sensor_values)` a cada ciclo, nao apenas acumular centro.
   - Ao finalizar, validar se cada sensor tem faixa minima aceitavel. Sensores sem faixa devem manter default ou ultima calibracao valida.
   - Definir polaridade uma vez ao final, evitando alternancia por leituras isoladas.

4. Portar a ideia do ES070 de nao atualizar min/max durante o seguimento
   - Remover/evitar qualquer atualizacao dinamica de min/max em `STATE_FOLLOWING`.
   - Atualizar apenas centro/erro filtrado durante seguimento.
   - Outliers devem ser filtrados antes da normalizacao, mas nao devem expandir min/max.

5. Reintroduzir thresholds normalizados como camada de aplicacao
   - Adicionar constantes equivalentes:
     - `LINE_CENTER_ACTIVE_THRESHOLD = 0.55f`
     - `LINE_SIDE_ACTIVE_THRESHOLD = 0.25f`
   - Usar esses thresholds em `LineSensor_GetState()` para detectar:
     - linha central
     - lateral esquerda/direita
     - linha perdida
     - cruzamento/fim de percurso
   - Usar `sensor_threshold` de `AppConfig_t` ou substituir por thresholds float configuraveis.

6. Ajustar aplicacao do movimento mantendo a arquitetura atual
   - Nao copiar o `controller_sensor()` inteiro do ES070.
   - Manter PID do ES670, mas alimentar o PID com erro calculado a partir dos sensores normalizados.
   - Para casos extremos, aplicar a mesma intencao do ES070:
     - linha so nos sensores direitos: reduzir roda esquerda ou pivotar para esquerda
     - linha so nos sensores esquerdos: reduzir roda direita ou pivotar para direita
     - nenhum sensor ativo por N ciclos: buscar linha e depois parar
     - todos ativos por N ciclos: tratar como cruzamento/fim de percurso

7. Ajustar leitura ADC de forma controlada
   - Avaliar portar oversampling do ES070 (`ADC_OVERSAMPLING_RATIO_16`, right shift 4) via CubeMX ou edicao protegida.
   - Se nao mudar `.ioc`, manter a media de software atual (`LINE_ADC_SAMPLES = 4`) e aumentar apenas se o tempo de ciclo de 20 ms continuar folgado.

8. Validacao
   - Criar modo debug que imprima raw, normalizado, min/max, centro, erro, estado e polaridade.
   - Testar quatro cenarios com o robo parado:
     - branco em todos os sensores
     - fita no centro
     - fita na esquerda
     - fita na direita
   - Testar pista:
     - reta
     - curva esquerda/direita
     - perda temporaria de linha
     - cruzamento/fim de percurso

## Ordem sugerida de implementacao

1. Implementar estrutura de calibracao e defaults em `Core/Src/line_sensor.c` e `Core/Inc/line_sensor.h`.
2. Adicionar API de normalizacao em lote.
3. Corrigir `STATE_CALIBRATING` em `Core/Src/app_tasks.c` para atualizar min/max durante a janela de calibracao.
4. Reescrever `LineSensor_GetCentroidIndex()`, `LineSensor_GetInterpolatedValue()` e `LineSensor_GetState()` para priorizar normalizados.
5. Atualizar debug UART para imprimir valores normalizados e min/max.
6. Ajustar constantes de PID/base speed somente depois de validar que erro e estado estao coerentes.
7. Opcionalmente portar oversampling ADC se o ruido continuar alto.

## Riscos

- Os valores fixos do ES070 nao podem ser copiados diretamente porque as escalas dos ADCs diferem.
- O ES670 usa a mesma instancia ADC2 para sensores e bateria; mudar configuracao global de ADC pode afetar leitura de bateria.
- Alterar `.ioc` pode regenerar codigo e sobrescrever trechos fora de `USER CODE`.
- A deteccao de polaridade deve ser feita com amostras representativas; definir polaridade por uma unica leitura pode inverter o erro.

## Criterio de pronto

- `STATE_DEBUG` mostra normalizados coerentes: branco perto de `0.0`, linha/preto perto de `1.0`.
- Em linha central, `LineSensor_GetInterpolatedValue()` fica proximo de `0.0`.
- Linha deslocada para esquerda gera erro com sinal consistente com a correcao de motor.
- Min/max nao mudam durante `STATE_FOLLOWING`.
- Perda de linha e cruzamento continuam funcionando com debounce.
