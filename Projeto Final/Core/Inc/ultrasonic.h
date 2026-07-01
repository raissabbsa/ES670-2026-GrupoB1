#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

#define ULTRASONIC_OBSTACLE_CM   10U
#define ULTRASONIC_OBSTACLE_DEBOUNCE  3U
#define ULTRASONIC_MAX_CM        400U

void Ultrasonic_Init(void);
void Ultrasonic_CaptureISR(void);
uint16_t Ultrasonic_ReadDistance(void);

#endif
