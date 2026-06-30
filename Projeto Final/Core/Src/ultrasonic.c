#include "ultrasonic.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;

static volatile uint32_t s_rise = 0;
static volatile uint32_t s_fall = 0;
static volatile uint8_t s_state = 0;

void Ultrasonic_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = Ultra_Trig_PWM_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Ultra_Trig_PWM_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(Ultra_Trig_PWM_GPIO_Port, Ultra_Trig_PWM_Pin, GPIO_PIN_RESET);
}

void Ultrasonic_CaptureISR(void)
{
    uint32_t val = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);

    if (s_state == 1) {
        s_rise = val;
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        s_state = 2;
    } else if (s_state == 2) {
        s_fall = val;
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
        s_state = 3;
    }
}

static void delay_us(uint32_t us)
{
    volatile uint32_t count = (SystemCoreClock / 5000000U) * us;
    while (count--) {
        __NOP();
    }
}

uint16_t Ultrasonic_ReadDistance(void)
{
    HAL_GPIO_WritePin(Ultra_Trig_PWM_GPIO_Port, Ultra_Trig_PWM_Pin, GPIO_PIN_SET);
    delay_us(12);
    HAL_GPIO_WritePin(Ultra_Trig_PWM_GPIO_Port, Ultra_Trig_PWM_Pin, GPIO_PIN_RESET);

    s_state = 1;
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

    uint32_t t0 = HAL_GetTick();
    while (s_state != 3 && (HAL_GetTick() - t0) < 30) {
        /* spin */
    }

    if (s_state != 3) {
        HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
        s_state = 0;
        return ULTRASONIC_MAX_CM;
    }

    uint32_t pulse;
    if (s_fall >= s_rise) {
        pulse = s_fall - s_rise;
    } else {
        pulse = (65535U - s_rise) + s_fall + 1U;
    }

    s_state = 0;
    uint16_t cm = (uint16_t)(pulse / 58U);
    return (cm > ULTRASONIC_MAX_CM) ? ULTRASONIC_MAX_CM : cm;
}
