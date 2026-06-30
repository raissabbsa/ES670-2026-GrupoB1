#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#define WHEEL_DIAMETER_CM    6.5f
#define WHEEL_BASE_CM        15.0f
#define WHEEL_CIRCUM_CM      (3.14159f * WHEEL_DIAMETER_CM)
#define CM_PER_PULSE         (WHEEL_CIRCUM_CM / 20.0f)

typedef struct {
    float distance_cm;
    float speed_cms;
    float heading_deg;
    float battery_pct;
    uint16_t obstacle_cm;
    uint8_t state;
} Telemetry_t;

void Telemetry_Init(void);
void Telemetry_Get(Telemetry_t *out);
void Telemetry_SetDistSpeed(float dist, float spd);
void Telemetry_SetHeading(float hdg);
void Telemetry_SetBattery(float bat);
void Telemetry_SetObstacle(uint16_t obs);
void Telemetry_SetState(uint8_t st);

#endif
