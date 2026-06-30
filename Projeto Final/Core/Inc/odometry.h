/**
 * @file odometry.h
 * @brief Odometria diferencial baseada nas contagens dos encoders.
 *
 * Mantém estado (x, y, theta) integrado a cada chamada de Odometry_Update().
 * Deve ser chamada periodicamente (50–100 Hz é suficiente). A integração usa
 * a fórmula de arco médio (cap. 3 da modelagem cinemática do prof.):
 *
 *     dL_left  = pulses_left  * CM_PER_PULSE
 *     dL_right = pulses_right * CM_PER_PULSE
 *     dL = (dL_left + dL_right) / 2
 *     dT = (dL_right - dL_left) / WHEEL_BASE_CM
 *     x  += dL * cos(theta + dT/2)
 *     y  += dL * sin(theta + dT/2)
 *     theta += dT
 *
 * Adicionado em feature/joao por João Santos.
 */
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "main.h"

/* Geometria do robô — confirmar/medir no protótipo real */
#define ODOMETRY_WHEEL_BASE_CM       14.0f
#define ODOMETRY_WHEEL_CM_PER_REV    22.0f
#define ODOMETRY_PULSES_PER_REV      20.0f
#define ODOMETRY_CM_PER_PULSE        (ODOMETRY_WHEEL_CM_PER_REV / ODOMETRY_PULSES_PER_REV)

void  Odometry_Init(void);
void  Odometry_Reset(void);
void  Odometry_Update(void);          /* chama Encoder_GetCount* internamente */

float Odometry_GetX(void);
float Odometry_GetY(void);
float Odometry_GetThetaDeg(void);
float Odometry_GetDistanceCm(void);   /* distância total percorrida acumulada */

#endif /* ODOMETRY_H */
