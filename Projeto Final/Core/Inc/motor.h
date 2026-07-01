#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* TIM1 configurado como no exemplo do professor: ARR = 999 */
#define MOTOR_PWM_MAX 999U
#define MOTOR_MIN_START_POWER 0.40f
#define MOTOR_MIN_TURN_POWER  0.30f

void Motor_Init(void);
void Motor_SetPower(float left, float right);

/* Garante minimo de partida em cada roda que estiver > 0 */
void Motor_SetPowerDiff(float left, float right);
void Motor_SetSpeed(int16_t left, int16_t right);
void Motor_Stop(void);
void Motor_Brake(void);

/* Escala de duty cycle por roda (1.0 = sem compensacao) */
void Motor_SetLeftScale(float scale);
void Motor_SetRightScale(float scale);
void Motor_GetScales(float *left, float *right);

#endif /* MOTOR_H */
