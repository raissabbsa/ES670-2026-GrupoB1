#include "encoder.h"
#include "main.h"

extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

#define ENCODER_MAX_DELTA_PER_PERIOD  5
#define ENCODER_DT                    0.01f

static volatile int32_t encoder_count_left = 0;
static volatile int32_t encoder_count_right = 0;
static int32_t last_count_left = 0;
static int32_t last_count_right = 0;
static float speed_left_rps = 0.0f;
static float speed_right_rps = 0.0f;

void Encoder_Init(void)
{
    HAL_TIM_IC_Start_IT(&htim16, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);
    encoder_count_left = 0;
    encoder_count_right = 0;
}

void Encoder_LeftPulseISR(void)
{
    encoder_count_left++;
}

void Encoder_RightPulseISR(void)
{
    encoder_count_right++;
}

void Encoder_Update(void)
{
    int32_t current_left = encoder_count_left;
    int32_t current_right = encoder_count_right;

    int32_t delta_left = current_left - last_count_left;
    int32_t delta_right = current_right - last_count_right;

    last_count_left = current_left;
    last_count_right = current_right;

    if (delta_left > ENCODER_MAX_DELTA_PER_PERIOD) {
        delta_left = ENCODER_MAX_DELTA_PER_PERIOD;
    }
    if (delta_right > ENCODER_MAX_DELTA_PER_PERIOD) {
        delta_right = ENCODER_MAX_DELTA_PER_PERIOD;
    }

    speed_left_rps = (float)delta_left / (float)ENCODER_PULSES_PER_REV / ENCODER_DT;
    speed_right_rps = (float)delta_right / (float)ENCODER_PULSES_PER_REV / ENCODER_DT;
}

int32_t Encoder_GetCountLeft(void)
{
    return encoder_count_left;
}

int32_t Encoder_GetCountRight(void)
{
    return encoder_count_right;
}

void Encoder_ResetCounts(void)
{
    encoder_count_left = 0;
    encoder_count_right = 0;
    last_count_left = 0;
    last_count_right = 0;
}

float Encoder_GetSpeedLeft(void)
{
    return speed_left_rps;
}

float Encoder_GetSpeedRight(void)
{
    return speed_right_rps;
}
