#ifndef APP_TASKS_H
#define APP_TASKS_H

#include <stdint.h>
#include "cmsis_os2.h"

/* Line follower states */
typedef enum {
    STATE_IDLE,
    STATE_ALIGNING,
    STATE_CALIBRATING,
    STATE_FOLLOWING,
    STATE_IN_CROSSING,
    STATE_STOPPING,
    STATE_STOPPED,
    STATE_DEBUG,
    STATE_MANUAL,
} LineFollower_State;

/* Motor command structure sent via queue */
typedef struct {
    float vel_left;
    float vel_right;
} MotorCmd_t;

/* Configurable parameters */
typedef struct {
    float line_Kp;
    float line_Ki;
    float line_Kd;
    float speed_Kp;
    float speed_Ki;
    float speed_Kd;
    float base_speed;
    uint16_t sensor_threshold;
    uint32_t max_time_ms;
} AppConfig_t;

extern AppConfig_t app_config;
extern volatile LineFollower_State follower_state;
extern volatile MotorCmd_t g_motor_cmd;

void App_LineCtrlTask(void *argument);
void App_MotorCtrlTask(void *argument);

#endif /* APP_TASKS_H */
