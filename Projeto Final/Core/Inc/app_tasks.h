#ifndef APP_TASKS_H
#define APP_TASKS_H

#include <stdint.h>
#include "cmsis_os2.h"

/* Fila de comandos de motor (definida em main.c, compartilhada com bluetooth). */
extern osMessageQueueId_t queueMotorCmdHandle;

/* Line follower states */
typedef enum {
    STATE_IDLE,
    STATE_ALIGNING,
    STATE_CALIBRATING,
    STATE_CALIB_SENSORES,
    STATE_CALIB_MOTORES,
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
/* Contador de ciclos do App_MotorCtrlTask. Utilizado para diagnostico
 * (verificar se a task esta rodando). Incrementa a cada iteracao (10ms). */
extern volatile uint32_t g_motor_cycles;

void App_LineCtrlTask(void *argument);
void App_MotorCtrlTask(void *argument);

/* Ajuste de PID em tempo real (Bluetooth $SET) */
void App_SetLinePidGains(float kp, float ki, float kd);
void App_SetBaseSpeed(float speed);
void App_GetLinePidConfig(float *kp, float *ki, float *kd, float *speed);

/* Leitura crua do ADC da bateria (0-4095). Usada para debug. */
uint16_t App_ReadBatteryRaw(void);

/* Inicia calibracao automatica de sensores (passiva). */
void App_StartCalibSensores(void);

/* Inicia calibracao automatica de motores (ativa). */
void App_StartCalibMotores(void);

#endif /* APP_TASKS_H */
