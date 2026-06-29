#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#define ENCODER_PULSES_PER_REV  20

void Encoder_Init(void);
void Encoder_Update(void);
int32_t Encoder_GetCountLeft(void);
int32_t Encoder_GetCountRight(void);
void Encoder_ResetCounts(void);
float Encoder_GetSpeedLeft(void);
float Encoder_GetSpeedRight(void);

void Encoder_LeftPulseISR(void);
void Encoder_RightPulseISR(void);

#endif /* ENCODER_H */
