/**
 * @file ultrasonic.h
 * @brief Driver do HC-SR04 (TIM20_CH1 trigger PWM + TIM3_CH1 echo input capture DMA).
 *
 * Adicionado em feature/joao por João Santos (sessão 29/06/2026).
 *
 * Uso típico:
 *   Ultrasonic_Init();                   // chamar em main USER CODE 2
 *   float d = Ultrasonic_GetDistanceCm(); // chamar em qualquer task
 *
 * Convenção de fora de range: devolve ULTRASONIC_OUT_OF_RANGE_CM (400 cm).
 */
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "main.h"

#define ULTRASONIC_OUT_OF_RANGE_CM   400.0f

void  Ultrasonic_Init(void);
float Ultrasonic_GetDistanceCm(void);

#endif /* ULTRASONIC_H */
