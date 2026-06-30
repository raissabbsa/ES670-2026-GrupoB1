/**
 * @file ultrasonic.c
 * @brief HC-SR04 driver. Trigger PWM contínuo em TIM20_CH1 (10us a cada ciclo,
 *        ~100Hz). Echo capturado por TIM3_CH1 com DMA para 2 timestamps
 *        (subida + descida). Distância = (t1 - t0) / 58.0 cm.
 *
 * Adicionado em feature/joao por João Santos.
 */

#include "ultrasonic.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim20;

#define ULTRASONIC_CAPTURE_COUNT          2U
#define ULTRASONIC_TRIGGER_COMPARE_VALUE  10U    /* TIM20 period=100 @1us/tick -> 10us */
#define ULTRASONIC_DISTANCE_DIVISOR       58.0f  /* us -> cm (HC-SR04 datasheet) */

static uint16_t g_capture[ULTRASONIC_CAPTURE_COUNT];

void Ultrasonic_Init(void)
{
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_1, ULTRASONIC_TRIGGER_COMPARE_VALUE);

    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_1,
                         (uint32_t *)g_capture,
                         ULTRASONIC_CAPTURE_COUNT);
}

float Ultrasonic_GetDistanceCm(void)
{
    uint16_t first  = g_capture[0];
    uint16_t second = g_capture[1];
    /* Subtração módulo-16-bits (HAL captures são uint16). */
    uint16_t delta  = (uint16_t)(second - first);

    float distance = (float)delta / ULTRASONIC_DISTANCE_DIVISOR;

    if (distance > ULTRASONIC_OUT_OF_RANGE_CM || distance < 0.0f) {
        distance = ULTRASONIC_OUT_OF_RANGE_CM;
    }
    return distance;
}
