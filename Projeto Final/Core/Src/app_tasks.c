#include "app_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "line_sensor.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "telemetry.h"
#include "ultrasonic.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

extern osMessageQueueId_t queueMotorCmdHandle;
extern UART_HandleTypeDef hlpuart1;

AppConfig_t app_config = {
    .line_Kp = 0.35f,
    .line_Ki = 0.0f,
    .line_Kd = 0.04f,
    .speed_Kp = 1.0f,
    .speed_Ki = 0.5f,
    .speed_Kd = 0.0f,
    .base_speed = 0.30f,
    .sensor_threshold = 0,
    .max_time_ms = 90000,
};

volatile LineFollower_State follower_state = STATE_IDLE;
volatile MotorCmd_t g_motor_cmd = {0.0f, 0.0f};
volatile uint32_t g_motor_cycles = 0U;

#define LINE_CORRECTION_MAX      0.18f
#define LINE_CALIBRATION_MS      1200U
#define LINE_CALIBRATION_SWEEP_MS 600U
#define LINE_CALIB_SWEEP_POWER    0.25f
#define LINE_LOST_STOP_COUNT     100U
#define LINE_LOST_DEBOUNCE       5U
#define FOLLOW_RAMP_MS           400U
#define LINE_LOST_SPREAD_MIN     8U
#define DEBUG_TX_INTERVAL_MS     500U
#define FOLLOW_TX_INTERVAL_MS    400U
#define LINE_OFFSET_MIN_SPREAD   38U
#define LINE_CENTER_VALID_MIN    1.5f
#define LINE_CENTER_VALID_MAX    3.5f
#define LINE_CENTER_MIN_SAMPLES  12U
#define FOLLOW_MAX_SPEED         0.55f
#define FOLLOW_MIN_SPEED         0.32f
#define FOLLOW_BOOST_MS          800U
#define FOLLOW_BOOST_SPEED       0.50f
#define LINE_CURVE_SLOWDOWN      0.12f
#define LINE_ERROR_FULL_SCALE    0.25f
#define LINE_CORRECTION_MAX_STRAIGHT 0.18f
#define LINE_CORRECTION_MAX_CURVE   0.30f
#define LINE_CORRECTION_SWITCH_ERR  0.15f
#define ALIGN_SPIN_POWER         0.35f
#define ALIGN_PIVOT_POWER        0.22f
#define ALIGN_LINE_SPREAD_MIN    25U
#define ALIGN_DETECT_STREAK      6U
#define ALIGN_OK_STREAK          12U
#define ALIGN_CREEP_MIN_MS       600U
#define ALIGN_CREEP_FAIL_MS      3000U
#define ALIGN_CENTER_ERR_MAX     0.08f
#define ALIGN_SPIN_PULSES_ROT    70
#define ALIGN_TOTAL_MAX_MS       8000U
#define CROSSING_STRAIGHT_MS     400U
#define ULTRASONIC_CHECK_CYCLES  5U

/* Calibracao automatica de sensores (passiva) */
#define CALIB_SENS_TIMEOUT_MS    30000U
#define CALIB_SENS_BLINK_MS      100U

/* Calibracao automatica de motores (ativa) */
#define CALIB_MOT_FWD_MS         2000U
#define CALIB_MOT_REV_MS         1000U
#define CALIB_MOT_PAUSE_MS       200U
#define CALIB_MOT_SPEED          0.30f
#define CALIB_MOT_TOTAL_MS       (CALIB_MOT_FWD_MS + CALIB_MOT_PAUSE_MS + \
                                  CALIB_MOT_REV_MS + CALIB_MOT_PAUSE_MS)

typedef enum {
    ALIGN_PHASE_SPIN = 0,
    ALIGN_PHASE_CREEP,
} AlignPhase_t;

static void App_ApplyMotorCmd(MotorCmd_t *cmd)
{
    g_motor_cmd = *cmd;
    osMessageQueuePut(queueMotorCmdHandle, cmd, 0, 0);
}

/* Garante potencia minima nas duas rodas em seguimento (preserva diferenca). */
static void App_BoostFollowWheels(MotorCmd_t *cmd)
{
    if (cmd->vel_left <= 0.0f || cmd->vel_right <= 0.0f) {
        return;
    }

    float lo = (cmd->vel_left < cmd->vel_right) ? cmd->vel_left : cmd->vel_right;
    if (lo < FOLLOW_MIN_SPEED) {
        float boost = FOLLOW_MIN_SPEED - lo;
        cmd->vel_left += boost;
        cmd->vel_right += boost;
        if (cmd->vel_left > FOLLOW_MAX_SPEED) {
            cmd->vel_left = FOLLOW_MAX_SPEED;
        }
        if (cmd->vel_right > FOLLOW_MAX_SPEED) {
            cmd->vel_right = FOLLOW_MAX_SPEED;
        }
    }
}

static uint8_t App_LineVisibleQuick(const uint16_t values[LINE_SENSOR_COUNT])
{
    if (LineSensor_GetRawSpread(values) < ALIGN_LINE_SPREAD_MIN) {
        return 0U;
    }

    return LineSensor_IsLineVisible(values);
}

static void App_AccumulateCenter(float center_idx, float *center_sum, uint32_t *center_count)
{
    if (center_idx < LINE_CENTER_VALID_MIN || center_idx > LINE_CENTER_VALID_MAX) {
        return;
    }

    if (*center_count == 0U) {
        *center_sum += center_idx;
        (*center_count)++;
        return;
    }

    float avg = *center_sum / (float)(*center_count);
    float delta = center_idx - avg;

    if (delta < 0.0f) {
        delta = -delta;
    }

    if (delta > 0.15f) {
        return;
    }

    *center_sum += center_idx;
    (*center_count)++;
}

static float App_ValidateCenter(float candidate, float fallback)
{
    if (candidate >= LINE_CENTER_VALID_MIN && candidate <= LINE_CENTER_VALID_MAX) {
        return candidate;
    }

    if (fallback >= LINE_CENTER_VALID_MIN && fallback <= LINE_CENTER_VALID_MAX) {
        return fallback;
    }

    return 2.6f;
}

static void App_DebugUartSend(const char *msg)
{
    if (msg == NULL) {
        return;
    }
    size_t len = strlen(msg);
    if (len > 0U) {
        HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, (uint16_t)len, 100);
    }
}

static PID_Controller s_line_pid;
static volatile uint8_t s_line_pid_gains_pending = 0U;

/* Variaveis compartilhadas entre App_StartCalibSensores() (externo) e
 * App_LineCtrlTask() para a calibracao passiva de sensores. */
static volatile uint32_t s_calib_sens_start_tick = 0U;
static volatile uint32_t s_calib_sens_last_blink = 0U;
static volatile uint8_t s_calib_sens_blink_state = 0U;

/* Variaveis compartilhadas entre App_StartCalibMotores() e
 * App_LineCtrlTask() para a calibracao ativa de motores. */
static volatile uint32_t s_calib_mot_start_tick = 0U;
static volatile uint8_t s_calib_mot_phase = 0U;
static volatile int32_t s_calib_mot_left_fwd = 0;
static volatile int32_t s_calib_mot_right_fwd = 0;

void App_SetLinePidGains(float kp, float ki, float kd)
{
    app_config.line_Kp = kp;
    app_config.line_Ki = ki;
    app_config.line_Kd = kd;
    s_line_pid_gains_pending = 1U;
}

void App_SetBaseSpeed(float speed)
{
    if (speed < 0.0f) {
        speed = 0.0f;
    }
    if (speed > 1.0f) {
        speed = 1.0f;
    }
    app_config.base_speed = speed;
}

void App_GetLinePidConfig(float *kp, float *ki, float *kd, float *speed)
{
    if (kp != NULL) {
        *kp = app_config.line_Kp;
    }
    if (ki != NULL) {
        *ki = app_config.line_Ki;
    }
    if (kd != NULL) {
        *kd = app_config.line_Kd;
    }
    if (speed != NULL) {
        *speed = app_config.base_speed;
    }
}

static void App_ApplyPendingLinePidGains(void)
{
    if (s_line_pid_gains_pending == 0U) {
        return;
    }

    PID_SetGains(&s_line_pid, app_config.line_Kp, app_config.line_Ki, app_config.line_Kd);
    s_line_pid_gains_pending = 0U;
}

/* Inicia calibracao automatica de sensores (passiva).
 * O usuario move o robo manualmente sobre a pista, e o firmware
 * captura min/max dos sensores. */
void App_StartCalibSensores(void)
{
    LineSensor_ResetCalibration();
    s_calib_sens_start_tick = xTaskGetTickCount();
    s_calib_sens_last_blink = 0U;
    s_calib_sens_blink_state = 0U;
    follower_state = STATE_CALIB_SENSORES;
    App_DebugUartSend("Calib S: capturando (mexa o robo)...\r\n");
}

/* Inicia calibracao automatica de motores (ativa).
 * O robo anda sozinho por ~3s e o firmware calcula LSCL/RSCL. */
void App_StartCalibMotores(void)
{
    if (follower_state != STATE_IDLE) {
        App_DebugUartSend("Calib M: estado nao eh IDLE\r\n");
        return;
    }
    s_calib_mot_start_tick = xTaskGetTickCount();
    s_calib_mot_phase = 0U;
    s_calib_mot_left_fwd = 0;
    s_calib_mot_right_fwd = 0;
    follower_state = STATE_CALIB_MOTORES;
    App_DebugUartSend("Calib M: andando (~3s)...\r\n");
}

void App_LineCtrlTask(void *argument)
{
    (void)argument;

    LineSensor_Init();

    App_DebugUartSend("Robo OK - serial 115200\r\n");
    App_DebugUartSend("Enter=inicia (alinha sozinho)\r\n");

    PID_Init(&s_line_pid, app_config.line_Kp, app_config.line_Ki,
             app_config.line_Kd, 0.02f, -LINE_CORRECTION_MAX_CURVE, LINE_CORRECTION_MAX_CURVE);

    uint16_t sensor_values[LINE_SENSOR_COUNT];
    MotorCmd_t cmd;
    float last_valid_error = 0.0f;
    uint32_t lost_counter = 0;
    uint32_t start_tick = 0;
    uint32_t calib_start_tick = 0;
    uint8_t started = 0;
    LineFollower_State prev_state = STATE_IDLE;
    uint32_t last_led_toggle = 0;
    uint32_t last_debug_tx = 0;
    uint32_t last_follow_tx = 0;
    float center_sum = 0.0f;
    uint32_t center_count = 0U;
    float last_good_center = 2.6f;
    uint8_t follow_msg_sent = 0;
    uint32_t align_start_tick = 0;
    uint32_t align_creep_start = 0;
    AlignPhase_t align_phase = ALIGN_PHASE_SPIN;
    int8_t align_spin_dir = 1;
    int32_t align_spin_pulse_start = 0;
    uint8_t align_detect_streak = 0;
    uint8_t align_ok_streak = 0;
    float filtered_line_error = 0.0f;
    uint32_t crossing_start_tick = 0;
    uint8_t crossing_count = 0;  /* 0=nenhum, 1=primeiro (cruzamento), 2+=chegada */
    uint32_t ultrasonic_cycle = 0;
    uint8_t obstacle_streak = 0;
    uint8_t line_lost_streak = 0;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));

        App_ApplyPendingLinePidGains();

        if (follower_state == STATE_DEBUG && prev_state != STATE_DEBUG) {
            last_debug_tx = 0;
            LineSensor_ReadAll(sensor_values);
            LineSensor_SetPolarity(
                LineSensor_DetectPolarity());
            App_DebugUartSend("=== MODO DEBUG SENSORES ===\r\n");
            App_DebugUartSend("Centro na linha: err~0 cen~ctr\r\n");
            App_DebugUartSend("Baixo=sair | Enter=iniciar seguir\r\n");
        }

        if (follower_state == STATE_IDLE && prev_state == STATE_DEBUG) {
            App_DebugUartSend("Debug OFF. Enter=iniciar seguir linha\r\n");
        }

        if (follower_state == STATE_ALIGNING && prev_state != STATE_ALIGNING) {
            align_start_tick = xTaskGetTickCount();
            align_creep_start = 0;
            align_spin_dir = 1;
            align_spin_pulse_start = 0;
            align_detect_streak = 0;
            align_ok_streak = 0;
            center_sum = 0.0f;
            center_count = 0U;
            Encoder_ResetCounts();

            LineSensor_ReadAll(sensor_values);
            if (App_LineVisibleQuick(sensor_values)) {
LineSensor_SetPolarity(
                LineSensor_DetectPolarity());
                align_phase = ALIGN_PHASE_CREEP;
                align_creep_start = xTaskGetTickCount();
                align_ok_streak = 0U;
                App_DebugUartSend("Ja na fita - centralizando\r\n");
            } else {
                align_phase = ALIGN_PHASE_SPIN;
                App_DebugUartSend("=== ALINHANDO ===\r\n");
            }
        }

        if (follower_state == STATE_CALIBRATING && prev_state != STATE_CALIBRATING) {
            calib_start_tick = xTaskGetTickCount();
            center_sum = 0.0f;
            center_count = 0U;
            LineSensor_ResetCalibration();
            LineSensor_ReadAll(sensor_values);
            LineSensor_SetPolarity(
                LineSensor_DetectPolarity());
            started = 0;
            last_follow_tx = 0;
            HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
            App_DebugUartSend("=== CALIBRANDO (1.2s) ===\r\n");
        }

        if (follower_state == STATE_FOLLOWING &&
            (prev_state == STATE_IDLE || prev_state == STATE_STOPPED ||
             prev_state == STATE_CALIBRATING || prev_state == STATE_ALIGNING)) {
            started = 0;
            lost_counter = 0;
            follow_msg_sent = 0;
            last_follow_tx = 0;
            filtered_line_error = 0.0f;
            line_lost_streak = 0;
            crossing_count = 0;
            PID_Reset(&s_line_pid);
        }
        prev_state = follower_state;

        switch (follower_state) {

        case STATE_IDLE:
        case STATE_STOPPED:
            cmd.vel_left = 0.0f;
            cmd.vel_right = 0.0f;
            g_motor_cmd = cmd;
            osMessageQueuePut(queueMotorCmdHandle, &cmd, 0, 0);
            continue;

        case STATE_ALIGNING: {
            uint32_t elapsed_ms = (xTaskGetTickCount() - align_start_tick) * portTICK_PERIOD_MS;
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

            if (now - last_led_toggle >= 200) {
                HAL_GPIO_TogglePin(LED_Y_GPIO_Port, LED_Y_Pin);
                last_led_toggle = now;
            }

            LineSensor_ReadAll(sensor_values);

            if (align_phase == ALIGN_PHASE_SPIN) {
                if (App_LineVisibleQuick(sensor_values)) {
                    align_detect_streak++;
                } else {
                    align_detect_streak = 0U;
                }

                if (align_detect_streak >= ALIGN_DETECT_STREAK) {
                    align_phase = ALIGN_PHASE_CREEP;
                    align_creep_start = xTaskGetTickCount();
                    align_ok_streak = 0U;
                    align_detect_streak = 0U;
                    LineSensor_SetPolarity(
                         LineSensor_DetectPolarity());
                    App_DebugUartSend("Fita vista - centralizando\r\n");
                } else {
                    int32_t spin_count = Encoder_GetCountRight() + Encoder_GetCountLeft();

                    if (align_spin_pulse_start == 0) {
                        align_spin_pulse_start = spin_count;
                    }

                    if ((spin_count - align_spin_pulse_start) >= ALIGN_SPIN_PULSES_ROT) {
                        align_spin_pulse_start = spin_count;
                        align_spin_dir = (int8_t)(-align_spin_dir);
                    }

                    if (align_spin_dir > 0) {
                        cmd.vel_left = 0.0f;
                        cmd.vel_right = ALIGN_SPIN_POWER;
                    } else {
                        cmd.vel_left = ALIGN_SPIN_POWER;
                        cmd.vel_right = 0.0f;
                    }
                }
            }

            if (align_phase == ALIGN_PHASE_CREEP) {
                uint32_t creep_ms = (xTaskGetTickCount() - align_creep_start) * portTICK_PERIOD_MS;
                float line_err = LineSensor_GetInterpolatedValue(sensor_values);
                float abs_corr = line_err;

                if (abs_corr < 0.0f) {
                    abs_corr = -abs_corr;
                }

                /* Calibracao NAO atualiza min/max durante o alinhamento.
                 * O pivot gera leituras fora da fita que poluem o range.
                 * A calibracao real acontece em STATE_CALIBRATING. */

                float pivot = line_err * 1.2f;
                if (pivot > ALIGN_PIVOT_POWER) {
                    pivot = ALIGN_PIVOT_POWER;
                }
                if (pivot < -ALIGN_PIVOT_POWER) {
                    pivot = -ALIGN_PIVOT_POWER;
                }

                if (abs_corr < 0.03f) {
                    cmd.vel_left = 0.0f;
                    cmd.vel_right = 0.0f;
                } else {
                    cmd.vel_left = pivot;
                    cmd.vel_right = -pivot;
                }

                if (creep_ms >= ALIGN_CREEP_MIN_MS && abs_corr < ALIGN_CENTER_ERR_MAX) {
                    align_ok_streak++;
                } else {
                    align_ok_streak = 0U;
                }

                if (align_ok_streak >= ALIGN_OK_STREAK) {
                    cmd.vel_left = 0.0f;
                    cmd.vel_right = 0.0f;
                    follower_state = STATE_CALIBRATING;
                    App_DebugUartSend("Centralizado OK\r\n");
                } else if (creep_ms >= ALIGN_CREEP_FAIL_MS) {
                    cmd.vel_left = 0.0f;
                    cmd.vel_right = 0.0f;
                    follower_state = STATE_CALIBRATING;
                    App_DebugUartSend("Centralizado timeout\r\n");
                }
            }

            if (elapsed_ms >= ALIGN_TOTAL_MAX_MS) {
                if (align_phase == ALIGN_PHASE_SPIN) {
                    LineSensor_SetPolarity(LineSensor_DetectPolarity());
                }
                cmd.vel_left = 0.0f;
                cmd.vel_right = 0.0f;
                follower_state = STATE_CALIBRATING;
                App_DebugUartSend("Alinhamento timeout\r\n");
            }

            App_ApplyMotorCmd(&cmd);
            continue;
        }

        case STATE_CALIBRATING: {
            uint32_t elapsed_ms = (xTaskGetTickCount() - calib_start_tick) * portTICK_PERIOD_MS;

            LineSensor_ReadAll(sensor_values);
            LineSensor_UpdateCalibration(sensor_values);

            uint16_t spread = LineSensor_GetRawSpread(sensor_values);

            if (spread >= LINE_OFFSET_MIN_SPREAD) {
                float cen = LineSensor_GetCentroidIndex(sensor_values);
                App_AccumulateCenter(cen, &center_sum, &center_count);
            }

            /* Fase 1 (0..LINE_CALIBRATION_SWEEP_MS): varre linha/branco girando devagar.
             * Fase 2 (apos): parado sobre a fita para estabilizar o centro. */
            if (elapsed_ms < LINE_CALIBRATION_SWEEP_MS) {
                cmd.vel_left = LINE_CALIB_SWEEP_POWER;
                cmd.vel_right = -LINE_CALIB_SWEEP_POWER;
            } else {
                cmd.vel_left = 0.0f;
                cmd.vel_right = 0.0f;
            }
            App_ApplyMotorCmd(&cmd);

            if (elapsed_ms >= LINE_CALIBRATION_MS) {
                LineSensor_SetPolarity(LineSensor_DetectPolarity());
                LineSensor_FinalizeCalibration();

                if (center_count >= LINE_CENTER_MIN_SAMPLES) {
                    float candidate = center_sum / (float)center_count;
                    float center = App_ValidateCenter(candidate, last_good_center);
                    LineSensor_SetCenterTarget(center);
                    last_good_center = center;
                }

                /* Log de diagnostico: mostra polaridade, media das leituras
                 * cruas e min/max finais. Util para saber se calibrou certo. */
                {
                    LinePolarity pol = LineSensor_GetPolarity();
                    const char *pol_str = (pol == LINE_POLARITY_DARK) ? "DARK" : "LIGHT";
                    uint16_t mn[LINE_SENSOR_COUNT];
                    uint16_t mx[LINE_SENSOR_COUNT];
                    for (uint8_t k = 0U; k < LINE_SENSOR_COUNT; k++) {
                        LineSensor_GetCalibrationBounds(k, &mn[k], &mx[k]);
                    }
                    char buf2[96];
                    int n = snprintf(buf2, sizeof(buf2),
                        "Calib: pol=%s mean=%.0f m=[%u,%u,%u,%u,%u]/[%u,%u,%u,%u,%u]\r\n",
                        pol_str,
                        LineSensor_GetCalibMeanRaw(),
                        (unsigned)mn[0], (unsigned)mn[1], (unsigned)mn[2],
                        (unsigned)mn[3], (unsigned)mn[4],
                        (unsigned)mx[0], (unsigned)mx[1], (unsigned)mx[2],
                        (unsigned)mx[3], (unsigned)mx[4]);
                    if (n > 0 && n < (int)sizeof(buf2)) {
                        App_DebugUartSend(buf2);
                    }
                }

                HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
                filtered_line_error = 0.0f;
                PID_Reset(&s_line_pid);
                follower_state = STATE_FOLLOWING;
            }
            continue;
        }

        case STATE_CALIB_SENSORES: {
            /* Calibracao passiva: usuario move o robo, firmware so capta. */
            uint32_t now = xTaskGetTickCount();
            uint32_t elapsed_ms = (now - s_calib_sens_start_tick) * portTICK_PERIOD_MS;

            /* LED pisca rapido para feedback visual */
            if (elapsed_ms - s_calib_sens_last_blink > CALIB_SENS_BLINK_MS) {
                s_calib_sens_last_blink = elapsed_ms;
                s_calib_sens_blink_state = !s_calib_sens_blink_state;
                HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, s_calib_sens_blink_state);
            }

            /* Timeout: finaliza automaticamente */
            if (elapsed_ms > CALIB_SENS_TIMEOUT_MS) {
                LineSensor_SetPolarity(LineSensor_DetectPolarity());
                LineSensor_FinalizeCalibration();
                char log_buf[96];
                LinePolarity pol = LineSensor_GetPolarity();
                const char *pol_str = (pol == LINE_POLARITY_DARK) ? "DARK" : "LIGHT";
                uint16_t mn[LINE_SENSOR_COUNT];
                uint16_t mx[LINE_SENSOR_COUNT];
                for (uint8_t k = 0U; k < LINE_SENSOR_COUNT; k++) {
                    LineSensor_GetCalibrationBounds(k, &mn[k], &mx[k]);
                }
                int n = snprintf(log_buf, sizeof(log_buf),
                    "Calib S OK (timeout): pol=%s mean=%.0f m=[%u,%u,%u,%u,%u]/[%u,%u,%u,%u,%u]\r\n",
                    pol_str, LineSensor_GetCalibMeanRaw(),
                    (unsigned)mn[0], (unsigned)mn[1], (unsigned)mn[2],
                    (unsigned)mn[3], (unsigned)mn[4],
                    (unsigned)mx[0], (unsigned)mx[1], (unsigned)mx[2],
                    (unsigned)mx[3], (unsigned)mx[4]);
                if (n > 0 && n < (int)sizeof(log_buf)) {
                    App_DebugUartSend(log_buf);
                }
                HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
                follower_state = STATE_IDLE;
                break;
            }

            /* Captura: le sensores e atualiza min/max */
            LineSensor_ReadAll(sensor_values);
            LineSensor_UpdateCalibration(sensor_values);

            /* Motores ficam parados (usuario move o robo manualmente) */
            cmd.vel_left = 0.0f;
            cmd.vel_right = 0.0f;
            g_motor_cmd = cmd;
            osMessageQueuePut(queueMotorCmdHandle, &cmd, 0, 0);
            continue;
        }

        case STATE_CALIB_MOTORES: {
            /* Calibracao ativa: robo anda sozinho, encoder mede. */
            uint32_t now = xTaskGetTickCount();
            uint32_t elapsed_ms = (now - s_calib_mot_start_tick) * portTICK_PERIOD_MS;

            /* LED pisca rapido para feedback visual */
            if (elapsed_ms - last_led_toggle > CALIB_SENS_BLINK_MS) {
                last_led_toggle = elapsed_ms;
                HAL_GPIO_TogglePin(LED_Y_GPIO_Port, LED_Y_Pin);
            }

            if (s_calib_mot_phase == 0U) {
                /* Inicio: reseta encoders e comeca a andar para frente */
                Encoder_ResetCounts();
                s_calib_mot_phase = 1U;
                App_DebugUartSend("Calib M: fwd...\r\n");
            }

            if (elapsed_ms < CALIB_MOT_FWD_MS) {
                /* Anda reto para frente */
                cmd.vel_left = CALIB_MOT_SPEED;
                cmd.vel_right = CALIB_MOT_SPEED;
            } else if (elapsed_ms < CALIB_MOT_FWD_MS + CALIB_MOT_PAUSE_MS) {
                /* Parada 1: captura pulsos do trecho de ida */
                cmd.vel_left = 0.0f;
                cmd.vel_right = 0.0f;
                if (s_calib_mot_phase == 1U) {
                    s_calib_mot_left_fwd = Encoder_GetCountLeft();
                    s_calib_mot_right_fwd = Encoder_GetCountRight();
                    s_calib_mot_phase = 2U;
                    App_DebugUartSend("Calib M: rev...\r\n");
                }
            } else if (elapsed_ms < CALIB_MOT_FWD_MS + CALIB_MOT_PAUSE_MS + CALIB_MOT_REV_MS) {
                /* Anda reto para tras */
                cmd.vel_left = -CALIB_MOT_SPEED;
                cmd.vel_right = -CALIB_MOT_SPEED;
            } else if (elapsed_ms < CALIB_MOT_TOTAL_MS) {
                /* Parada 2: captura pulsos do trecho de volta e calcula */
                cmd.vel_left = 0.0f;
                cmd.vel_right = 0.0f;
                if (s_calib_mot_phase == 2U) {
                    int32_t left_rev = Encoder_GetCountLeft() - s_calib_mot_left_fwd;
                    int32_t right_rev = Encoder_GetCountRight() - s_calib_mot_right_fwd;
                    if (left_rev < 0) left_rev = -left_rev;
                    if (right_rev < 0) right_rev = -right_rev;
                    int32_t total_left = s_calib_mot_left_fwd + left_rev;
                    int32_t total_right = s_calib_mot_right_fwd + right_rev;

                    if (total_right > 0) {
                        float ratio = (float)total_left / (float)total_right;
                        float new_lscl = 1.0f;
                        float new_rscl = 1.0f;
                        if (ratio > 1.0f) {
                            new_lscl = 1.0f / ratio;
                        } else if (ratio > 0.0f) {
                            new_rscl = ratio;
                        }
                        if (new_lscl < 0.5f) new_lscl = 0.5f;
                        if (new_lscl > 1.5f) new_lscl = 1.5f;
                        if (new_rscl < 0.5f) new_rscl = 0.5f;
                        if (new_rscl > 1.5f) new_rscl = 1.5f;
                        Motor_SetLeftScale(new_lscl);
                        Motor_SetRightScale(new_rscl);

                        char log_buf[96];
                        int n = snprintf(log_buf, sizeof(log_buf),
                            "Calib M OK: L=%ld R=%ld ratio=%.2f LSCL=%.2f RSCL=%.2f\r\n",
                            (long)total_left, (long)total_right, ratio,
                            new_lscl, new_rscl);
                        if (n > 0 && n < (int)sizeof(log_buf)) {
                            App_DebugUartSend(log_buf);
                        }
                    } else {
                        App_DebugUartSend("Calib M FAIL: encoders nao geraram pulsos\r\n");
                    }
                    HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
                    s_calib_mot_phase = 3U;
                    follower_state = STATE_IDLE;
                    break;
                }
            } else {
                /* Timeout de seguranca */
                HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
                follower_state = STATE_IDLE;
                break;
            }

            App_ApplyMotorCmd(&cmd);
            continue;
        }

        case STATE_DEBUG: {
            cmd.vel_left = 0.0f;
            cmd.vel_right = 0.0f;
            g_motor_cmd = cmd;
            osMessageQueuePut(queueMotorCmdHandle, &cmd, 0, 0);

            LineSensor_ReadAll(sensor_values);

            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - last_debug_tx >= DEBUG_TX_INTERVAL_MS) {
                last_debug_tx = now;
                char buf[256];
                float norm[LINE_SENSOR_COUNT];
                uint16_t min2 = 0U;
                uint16_t max2 = 0U;
                float line_err = LineSensor_GetInterpolatedValue(sensor_values);
                LineSensor_GetNormalizedAll(sensor_values, norm);
                LineSensor_GetCalibrationBounds(2U, &min2, &max2);
                int len = snprintf(buf, sizeof(buf),
                    "IR[%3u %3u %3u %3u %3u] N[%02d %02d %02d %02d %02d] spr=%u err=%d cen=%d ctr=%d act=%u cal=%u m2=%u/%u\r\n",
                    sensor_values[0], sensor_values[1], sensor_values[2],
                    sensor_values[3], sensor_values[4],
                    (int)(norm[0] * 99.0f), (int)(norm[1] * 99.0f),
                    (int)(norm[2] * 99.0f), (int)(norm[3] * 99.0f),
                    (int)(norm[4] * 99.0f),
                    (unsigned)LineSensor_GetRawSpread(sensor_values),
                    (int)(line_err * 100.0f),
                    (int)(LineSensor_GetCentroidIndex(sensor_values) * 10.0f),
                    (int)(LineSensor_GetCenterTarget() * 10.0f),
                    (unsigned)LineSensor_GetActiveCount(sensor_values),
                    (unsigned)LineSensor_IsCalibrationValid(),
                    (unsigned)min2,
                    (unsigned)max2);
                if (len > 0 && len < (int)sizeof(buf)) {
                    App_DebugUartSend(buf);
                }
            }
            continue;
        }

        case STATE_STOPPING:
            cmd.vel_left = 0.0f;
            cmd.vel_right = 0.0f;
            g_motor_cmd = cmd;
            osMessageQueuePut(queueMotorCmdHandle, &cmd, 0, 0);
            Motor_Brake();
            follower_state = STATE_STOPPED;
            PID_Reset(&s_line_pid);
            App_DebugUartSend("=== PAROU ===\r\n");
            Telemetry_SetState((uint8_t)STATE_STOPPED);
            continue;

        case STATE_IN_CROSSING: {
            /* Cruzamento: andar reto ate atravessar a fita perpendicular.
             * Mantem velocidade base, sem correcao do PID. */
            uint32_t cross_elapsed = (xTaskGetTickCount() - crossing_start_tick) * portTICK_PERIOD_MS;
            cmd.vel_left = app_config.base_speed;
            cmd.vel_right = app_config.base_speed;
            App_ApplyMotorCmd(&cmd);

            if (cross_elapsed >= CROSSING_STRAIGHT_MS) {
                follower_state = STATE_FOLLOWING;
                lost_counter = 0;
                PID_Reset(&s_line_pid);
                filtered_line_error = 0.0f;
                App_DebugUartSend("Cruzamento OK\r\n");
            }
            continue;
        }

        case STATE_MANUAL:
            cmd = g_motor_cmd;
            if (cmd.vel_left <= 0.0f && cmd.vel_right <= 0.0f) {
                Motor_Stop();
            } else {
                Motor_SetPowerDiff(cmd.vel_left, cmd.vel_right);
            }
            continue;

        default:
            break;
        }

        /* === STATE_FOLLOWING === */

        if (!started) {
            start_tick = xTaskGetTickCount();
            started = 1;
        }

        if (!follow_msg_sent) {
            follow_msg_sent = 1;
            App_DebugUartSend("=== SEGUINDO LINHA ===\r\n");
            Telemetry_SetState((uint8_t)STATE_FOLLOWING);
        }

        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if ((xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS >= app_config.max_time_ms) {
            follower_state = STATE_STOPPING;
            continue;
        }

        LineSensor_ReadAll(sensor_values);

        /* Ultrasonic safety check every N cycles */
        ultrasonic_cycle++;
        if (ultrasonic_cycle >= ULTRASONIC_CHECK_CYCLES) {
            ultrasonic_cycle = 0;
            uint16_t dist = Ultrasonic_ReadDistance();
            Telemetry_SetObstacle(dist);
            if (dist < ULTRASONIC_OBSTACLE_CM) {
                if (obstacle_streak < UINT8_MAX) obstacle_streak++;
                if (obstacle_streak >= ULTRASONIC_OBSTACLE_DEBOUNCE) {
                    HAL_GPIO_WritePin(Buzzer_PWM_GPIO_Port, Buzzer_PWM_Pin, GPIO_PIN_SET);
                    follower_state = STATE_STOPPING;
                    App_DebugUartSend("!!! OBSTACULO !!!\r\n");
                    continue;
                }
            } else {
                obstacle_streak = 0U;
                HAL_GPIO_WritePin(Buzzer_PWM_GPIO_Port, Buzzer_PWM_Pin, GPIO_PIN_RESET);
            }
        }

        /* Deteccao de cruzamento e fim de percurso.
         * 1o evento LINE_CROSSING: cruzamento (continua reto).
         * 2o+ evento (LINE_CROSSING ou LINE_FINISHED): linha de chegada
         * (para imediatamente).
         *
         * LINE_CROSSING: 4 sensores ativos, spread alto = fita estreita.
         * LINE_FINISHED: 5 sensores ativos, spread baixo = fita larga. */
        LineSensor_State line_state = LineSensor_GetState(sensor_values);
        if (line_state == LINE_FINISHED || line_state == LINE_CROSSING) {
            crossing_count++;
            char log_buf[64];
            if (line_state == LINE_FINISHED) {
                snprintf(log_buf, sizeof(log_buf),
                    "Fim de percurso (fita larga, evento %d)\r\n",
                    crossing_count);
            } else {
                snprintf(log_buf, sizeof(log_buf),
                    "Cruzamento (evento %d)\r\n", crossing_count);
            }
            App_DebugUartSend(log_buf);

            if (crossing_count >= 2U) {
                /* Segundo evento: linha de chegada. Para. */
                Motor_Brake();
                g_motor_cmd.vel_left = 0.0f;
                g_motor_cmd.vel_right = 0.0f;
                HAL_GPIO_WritePin(Buzzer_PWM_GPIO_Port, Buzzer_PWM_Pin, GPIO_PIN_SET);
                follower_state = STATE_STOPPING;
                App_DebugUartSend("=== FIM DE PERCURSO ===\r\n");
            } else {
                /* Primeiro evento: cruzamento, continua reto. */
                crossing_start_tick = xTaskGetTickCount();
                follower_state = STATE_IN_CROSSING;
                PID_Reset(&s_line_pid);
                filtered_line_error = 0.0f;
            }
            continue;
        }

        uint16_t spread = LineSensor_GetRawSpread(sensor_values);
        float raw_error = LineSensor_GetInterpolatedValue(sensor_values);

        if (line_state == LINE_ON_TRACK || line_state == LINE_ON_TRACK_LOW_CONTRAST) {
            line_lost_streak = 0U;
            lost_counter = 0;

            filtered_line_error = filtered_line_error * 0.6f + raw_error * 0.4f;
            last_valid_error = filtered_line_error;

            float correction = PID_Compute(&s_line_pid, 0.0f, filtered_line_error);

            uint32_t follow_elapsed = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
            if (follow_elapsed < FOLLOW_RAMP_MS) {
                correction *= (float)follow_elapsed / (float)FOLLOW_RAMP_MS;
            }

            float base = app_config.base_speed;
            if (follow_elapsed < FOLLOW_BOOST_MS) {
                base = FOLLOW_BOOST_SPEED;
            }

            float abs_err = filtered_line_error;
            if (abs_err < 0.0f) {
                abs_err = -abs_err;
            }

            /* Velocidade adaptativa + correcao maxima adaptativa:
             * Em reta (|err| < 0.15): base=SPD, max_corr=0.18 (estavel).
             * Em curva (|err| >= 0.15): base=0.18, max_corr=0.30 (permite pivot).
             * Isso resolve o dilema "KP alto oscila em reta / KP baixo nao faz curva". */
            if (abs_err >= LINE_CORRECTION_SWITCH_ERR && base > LINE_CURVE_SLOWDOWN) {
                base = LINE_CURVE_SLOWDOWN;
            }
            float max_corr = (abs_err >= LINE_CORRECTION_SWITCH_ERR)
                             ? LINE_CORRECTION_MAX_CURVE
                             : LINE_CORRECTION_MAX_STRAIGHT;
            if (correction > max_corr) correction = max_corr;
            if (correction < -max_corr) correction = -max_corr;

            cmd.vel_left = base - correction;
            cmd.vel_right = base + correction;

            if (cmd.vel_left < 0.0f) cmd.vel_left = 0.0f;
            if (cmd.vel_right < 0.0f) cmd.vel_right = 0.0f;
            if (cmd.vel_left > FOLLOW_MAX_SPEED) cmd.vel_left = FOLLOW_MAX_SPEED;
            if (cmd.vel_right > FOLLOW_MAX_SPEED) cmd.vel_right = FOLLOW_MAX_SPEED;

            App_BoostFollowWheels(&cmd);
        } else if (line_lost_streak < LINE_LOST_DEBOUNCE) {
            /* Glitch de sensor (spr=0 momentaneo): continua com ultimo erro */
            line_lost_streak++;
            filtered_line_error = last_valid_error;

            float correction = PID_Compute(&s_line_pid, 0.0f, filtered_line_error);
            float base = app_config.base_speed;

            cmd.vel_left = base - correction;
            cmd.vel_right = base + correction;

            if (cmd.vel_left < 0.0f) cmd.vel_left = 0.0f;
            if (cmd.vel_right < 0.0f) cmd.vel_right = 0.0f;
            if (cmd.vel_left > FOLLOW_MAX_SPEED) cmd.vel_left = FOLLOW_MAX_SPEED;
            if (cmd.vel_right > FOLLOW_MAX_SPEED) cmd.vel_right = FOLLOW_MAX_SPEED;

            App_BoostFollowWheels(&cmd);
        } else {
            /* === LINHA PERDIDA: dead reckoning (estilo ES070) ===
             * Sem pivot, sem arco de busca. Continua com o ultimo erro
             * filtrado ate LINE_LOST_STOP_COUNT ciclos (2s), depois para. */
            line_lost_streak++;
            lost_counter++;

            if (lost_counter > LINE_LOST_STOP_COUNT) {
                follower_state = STATE_STOPPING;
                continue;
            }

            filtered_line_error = last_valid_error;

            float correction = PID_Compute(&s_line_pid, 0.0f, filtered_line_error);
            float base = app_config.base_speed;

            cmd.vel_left = base - correction;
            cmd.vel_right = base + correction;

            if (cmd.vel_left < 0.0f) cmd.vel_left = 0.0f;
            if (cmd.vel_right < 0.0f) cmd.vel_right = 0.0f;
            if (cmd.vel_left > FOLLOW_MAX_SPEED) cmd.vel_left = FOLLOW_MAX_SPEED;
            if (cmd.vel_right > FOLLOW_MAX_SPEED) cmd.vel_right = FOLLOW_MAX_SPEED;

            App_BoostFollowWheels(&cmd);
        }

        App_ApplyMotorCmd(&cmd);

        if (now_ms - last_follow_tx >= FOLLOW_TX_INTERVAL_MS) {
            last_follow_tx = now_ms;
            char buf[128];
            int len = snprintf(buf, sizeof(buf),
                "F %s vl=%d vr=%d adj=%d spr=%u\r\n",
                (lost_counter > 0 && line_lost_streak >= LINE_LOST_DEBOUNCE) ? "BUSCA" : "ON",
                (int)(cmd.vel_left * 100.0f),
                (int)(cmd.vel_right * 100.0f),
                (int)(filtered_line_error * 100.0f),
                (unsigned)spread);
            if (len > 0 && len < (int)sizeof(buf)) {
                App_DebugUartSend(buf);
            }
        }
    }
}

extern ADC_HandleTypeDef hadc2;

/* Leitura crua do ADC da bateria (0-4095) para debug. */
uint16_t App_ReadBatteryRaw(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    HAL_ADC_ConfigChannel(&hadc2, &sConfig);
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 10);
    uint32_t raw = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
    return (uint16_t)raw;
}

static float App_ReadBatteryPct(void)
{
    uint32_t raw = App_ReadBatteryRaw();

    /* Divisor resistivo: Vbat -> R1/R2 -> ADC (3.3V ref, 12-bit)
       Ajustar a escala conforme o divisor real do robo.
       Assumindo: 2 celulas LiPo (6.0V~8.4V), divisor 1:2 -> 3.0~4.2V no ADC */
    float voltage = ((float)raw / 4095.0f) * 3.3f * 2.0f;
    float pct = (voltage - 6.0f) / (8.4f - 6.0f) * 100.0f;
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
}

void App_MotorCtrlTask(void *argument)
{
    (void)argument;

    Encoder_Init();

    float total_dist_left = 0.0f;
    float total_dist_right = 0.0f;
    float heading_deg = 0.0f;
    int32_t prev_left = 0;
    int32_t prev_right = 0;
    uint32_t telem_counter = 0;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));

        g_motor_cycles++;

        /* Botao de panico: se o switch frontal for pressionado, brake imediato
         * e vai para STATE_STOPPING. Funciona em qualquer estado. */
        if (HAL_GPIO_ReadPin(Switch_Fr_GPIO_Port, Switch_Fr_Pin) == GPIO_PIN_SET) {
            Motor_Brake();
            g_motor_cmd.vel_left = 0.0f;
            g_motor_cmd.vel_right = 0.0f;
            follower_state = STATE_STOPPING;
            App_DebugUartSend("!!! PANICO !!!\r\n");
            continue;
        }

        Encoder_Update();

        int32_t cur_left = Encoder_GetCountLeft();
        int32_t cur_right = Encoder_GetCountRight();
        int32_t dl = cur_left - prev_left;
        int32_t dr = cur_right - prev_right;
        prev_left = cur_left;
        prev_right = cur_right;

        float dist_l = (float)dl * CM_PER_PULSE;
        float dist_r = (float)dr * CM_PER_PULSE;
        total_dist_left += dist_l;
        total_dist_right += dist_r;

        heading_deg += (dist_r - dist_l) / WHEEL_BASE_CM * (180.0f / 3.14159f);

        telem_counter++;
        if (telem_counter >= 10) {
            telem_counter = 0;
            float avg_dist = (total_dist_left + total_dist_right) / 2.0f;
            float avg_speed = ((Encoder_GetSpeedLeft() + Encoder_GetSpeedRight()) / 2.0f)
                              * WHEEL_CIRCUM_CM;
            Telemetry_SetDistSpeed(avg_dist, avg_speed);
            Telemetry_SetHeading(heading_deg);
            Telemetry_SetBattery(App_ReadBatteryPct());
        }

        MotorCmd_t cmd = g_motor_cmd;

        if (cmd.vel_left <= 0.0f && cmd.vel_right <= 0.0f) {
            Motor_Stop();
            continue;
        }

        Motor_SetPowerDiff(cmd.vel_left, cmd.vel_right);
    }
}
