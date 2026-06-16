#include "encoders.h"
#include "stm32g4xx_hal.h"

extern TIM_HandleTypeDef htim16, htim17;

encoder_t g_encL = {0};
encoder_t g_encR = {0};

/* Clock de tick do timer (Hz), calculado em ENC_Init a partir do PSC atual.
 * TIM16/TIM17 ficam no APB2; com APB2 prescaler = 1 (config atual do clock
 * do projeto, 170 MHz em todo o barramento), o clock do timer é igual a
 * PCLK2, sem o fator x2 do RM que se aplicaria se o prescaler do APB2 fosse
 * diferente de 1. */
static uint32_t g_encTimClkHz = 0;

void ENC_Init(void)
{
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    g_encTimClkHz = pclk2 / (htim16.Init.Prescaler + 1);

    /* Habilita captura e interrupção dos dois timers */
    HAL_TIM_IC_Start_IT(&htim16, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);

    /* Habilita a interrupção de OVERFLOW (update event) também,
     * para detectar rodas paradas e estender o alcance do período medido. */
    __HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim17, TIM_IT_UPDATE);
}

/* =====================================================================
 *   Callbacks chamadas pela HAL.
 *   Mantenha o trabalho aqui dentro O MAIS CURTO POSSÍVEL.
 * =====================================================================*/

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    encoder_t *enc;
    uint16_t cap;

    if (htim->Instance == TIM16) {
        enc = &g_encL;
        cap = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    } else if (htim->Instance == TIM17) {
        enc = &g_encR;
        cap = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    } else {
        return;
    }

    /* Período = (overflow * 65536 + cap_atual) - cap_anterior  (em ticks do timer) */
    uint32_t dt;
    if (enc->overflow_cnt == 0) {
        dt = (uint32_t)((uint16_t)(cap - enc->last_capture));
    } else {
        dt = (uint32_t)enc->overflow_cnt * 65536u
           + (uint32_t)cap - (uint32_t)enc->last_capture;
    }

    enc->period_ticks = dt;
    enc->last_capture = cap;
    enc->overflow_cnt = 0;
    enc->last_tick    = HAL_GetTick();
    enc->pulse_count++;
}

/* Overflow → roda muito lenta ou parada */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM16) {
        if (g_encL.overflow_cnt < 0xFFFF) g_encL.overflow_cnt++;
    } else if (htim->Instance == TIM17) {
        if (g_encR.overflow_cnt < 0xFFFF) g_encR.overflow_cnt++;
    }
}

/* =====================================================================
 *   Atualização periódica chamada pela task.
 * =====================================================================*/

static void update_one(encoder_t *enc)
{
    uint32_t now = HAL_GetTick();

    if ((now - enc->last_tick) > ENC_TIMEOUT_MS || enc->period_ticks == 0) {
        enc->rpm       = 0.0f;
        enc->speed_mps = 0.0f;
        return;
    }

    /* f = clock_tick / período_em_ticks
     *                   =>  RPM = 60 * f / pulsos_por_rev
     *                   =>  v   = M_PER_PULSE * f             */
    float freq_hz = (float)g_encTimClkHz / (float)enc->period_ticks;
    enc->rpm       = (60.0f * freq_hz) / (float)ENC_PULSES_PER_REV;
    enc->speed_mps = M_PER_PULSE * freq_hz;
}

void ENC_Update(void)
{
    update_one(&g_encL);
    update_one(&g_encR);
}

float ENC_GetSpeedL(void)     { return g_encL.speed_mps; }
float ENC_GetSpeedR(void)     { return g_encR.speed_mps; }
int32_t ENC_GetPulsesL(void)  { return (int32_t)g_encL.pulse_count; }
int32_t ENC_GetPulsesR(void)  { return (int32_t)g_encR.pulse_count; }

void ENC_ResetPulses(void)
{
    g_encL.pulse_count = 0;
    g_encR.pulse_count = 0;
}
