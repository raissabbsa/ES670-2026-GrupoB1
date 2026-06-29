#include "app_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "line_sensor.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

extern osMessageQueueId_t queueMotorCmdHandle;
extern UART_HandleTypeDef hlpuart1;

AppConfig_t app_config = {
    .line_Kp = 0.90f,
    .line_Ki = 0.0f,
    .line_Kd = 0.20f,
    .speed_Kp = 1.0f,
    .speed_Ki = 0.5f,
    .speed_Kd = 0.0f,
    .base_speed = 0.40f,
    .sensor_threshold = 0,
    .max_time_ms = 90000,
};

volatile LineFollower_State follower_state = STATE_IDLE;
volatile MotorCmd_t g_motor_cmd = {0.0f, 0.0f};

#define LINE_CORRECTION_MAX      0.20f
#define LINE_CALIBRATION_MS      650U
#define LINE_LOST_STOP_COUNT     500U
#define LINE_LOST_SPREAD_MIN     8U
#define DEBUG_TX_INTERVAL_MS     500U
#define FOLLOW_TX_INTERVAL_MS    400U
#define LINE_OFFSET_MIN_SPREAD   38U
#define LINE_CENTER_VALID_MIN    2.2f
#define LINE_CENTER_VALID_MAX    2.9f
#define LINE_CENTER_MIN_SAMPLES  12U
#define FOLLOW_MAX_SPEED         0.55f
#define FOLLOW_MIN_SPEED         0.30f
#define FOLLOW_BOOST_MS          600U
#define FOLLOW_BOOST_SPEED       0.48f
#define LINE_CURVE_SLOWDOWN      0.40f
#define LINE_ERROR_FULL_SCALE    0.25f
#define LINE_SEARCH_POWER        0.35f
#define ALIGN_SPIN_POWER         0.20f
#define ALIGN_PIVOT_POWER        0.22f
#define ALIGN_LINE_SPREAD_MIN    25U
#define ALIGN_DETECT_STREAK      6U
#define ALIGN_OK_STREAK          12U
#define ALIGN_CREEP_MIN_MS       600U
#define ALIGN_CREEP_FAIL_MS      3000U
#define ALIGN_CENTER_ERR_MAX     0.08f
#define ALIGN_SPIN_PULSES_ROT    70
#define ALIGN_TOTAL_MAX_MS       8000U

typedef enum {
    ALIGN_PHASE_SPIN = 0,
    ALIGN_PHASE_CREEP,
} AlignPhase_t;

static void App_ApplyMotorCmd(MotorCmd_t *cmd)
{
    g_motor_cmd = *cmd;
    osMessageQueuePut(queueMotorCmdHandle, cmd, 0, 0);
}

static uint8_t App_GetLineSensorIndex(const uint16_t values[LINE_SENSOR_COUNT])
{
    uint8_t idx = 0U;
    LinePolarity pol = LineSensor_GetPolarity();

    for (uint8_t i = 1U; i < LINE_SENSOR_COUNT; i++) {
        if (pol == LINE_POLARITY_DARK) {
            if (values[i] < values[idx]) {
                idx = i;
            }
        } else if (values[i] > values[idx]) {
            idx = i;
        }
    }

    return idx;
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

void App_LineCtrlTask(void *argument)
{
    (void)argument;

    LineSensor_Init();

    App_DebugUartSend("Robo OK - serial 115200\r\n");
    App_DebugUartSend("Enter=inicia (alinha sozinho)\r\n");

    PID_Controller line_pid;
    PID_Init(&line_pid, app_config.line_Kp, app_config.line_Ki,
             app_config.line_Kd, 0.02f, -LINE_CORRECTION_MAX, LINE_CORRECTION_MAX);

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
    uint8_t calib_polarity_done = 0;
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

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));

        if (follower_state == STATE_DEBUG && prev_state != STATE_DEBUG) {
            last_debug_tx = 0;
            LineSensor_ReadAll(sensor_values);
            LineSensor_SetPolarity(
                LineSensor_DetectPolarityFromLayout(sensor_values));
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
                    LineSensor_DetectPolarityFromLayout(sensor_values));
                calib_polarity_done = 1;
                follower_state = STATE_CALIBRATING;
                App_DebugUartSend("Ja na fita - calibrando\r\n");
            } else {
                align_phase = ALIGN_PHASE_SPIN;
                App_DebugUartSend("=== ALINHANDO ===\r\n");
            }
        }

        if (follower_state == STATE_CALIBRATING && prev_state != STATE_CALIBRATING) {
            calib_start_tick = xTaskGetTickCount();
            calib_polarity_done = 0;
            center_sum = 0.0f;
            center_count = 0U;
            LineSensor_ReadAll(sensor_values);
            LineSensor_SetPolarity(
                LineSensor_DetectPolarityFromLayout(sensor_values));
            calib_polarity_done = 1;
            started = 0;
            last_follow_tx = 0;
            HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
            App_DebugUartSend("=== CALIBRANDO (0.6s) ===\r\n");
        }

        if (follower_state == STATE_FOLLOWING &&
            (prev_state == STATE_IDLE || prev_state == STATE_STOPPED ||
             prev_state == STATE_CALIBRATING || prev_state == STATE_ALIGNING)) {
            started = 0;
            lost_counter = 0;
            follow_msg_sent = 0;
            last_follow_tx = 0;
            filtered_line_error = 0.0f;
            PID_Reset(&line_pid);
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
                        LineSensor_DetectPolarityFromLayout(sensor_values));
                    calib_polarity_done = 1;
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

                LineSensor_UpdateCalibration(sensor_values);

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
                    LineSensor_SetPolarity(LineSensor_DetectPolarityFromLayout(sensor_values));
                    calib_polarity_done = 1;
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

            uint16_t spread = LineSensor_GetRawSpread(sensor_values);

            if (spread >= LINE_OFFSET_MIN_SPREAD) {
                float cen = LineSensor_GetCentroidIndex(sensor_values);
                App_AccumulateCenter(cen, &center_sum, &center_count);
            }

            cmd.vel_left = 0.0f;
            cmd.vel_right = 0.0f;
            App_ApplyMotorCmd(&cmd);

            if (elapsed_ms >= LINE_CALIBRATION_MS) {
                if (center_count >= LINE_CENTER_MIN_SAMPLES) {
                    float candidate = center_sum / (float)center_count;
                    float center = App_ValidateCenter(candidate, last_good_center);
                    LineSensor_SetCenterTarget(center);
                    last_good_center = center;
                }

                HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
                filtered_line_error = 0.0f;
                PID_Reset(&line_pid);
                follower_state = STATE_FOLLOWING;
            }
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
                char buf[192];
                float line_err = LineSensor_GetInterpolatedValue(sensor_values);
                int len = snprintf(buf, sizeof(buf),
                    "IR[%3u %3u %3u %3u %3u] spr=%u err=%d cen=%d ctr=%d idx=%u act=%u\r\n",
                    sensor_values[0], sensor_values[1], sensor_values[2],
                    sensor_values[3], sensor_values[4],
                    (unsigned)LineSensor_GetRawSpread(sensor_values),
                    (int)(line_err * 100.0f),
                    (int)(LineSensor_GetCentroidIndex(sensor_values) * 10.0f),
                    (int)(LineSensor_GetCenterTarget() * 10.0f),
                    (unsigned)App_GetLineSensorIndex(sensor_values),
                    (unsigned)LineSensor_GetActiveCount(sensor_values));
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
            follower_state = STATE_STOPPED;
            PID_Reset(&line_pid);
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
        }

        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if ((xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS >= app_config.max_time_ms) {
            follower_state = STATE_STOPPING;
            continue;
        }

        LineSensor_ReadAll(sensor_values);

        uint16_t spread = LineSensor_GetRawSpread(sensor_values);
        float raw_error = LineSensor_GetInterpolatedValue(sensor_values);

        if (spread >= LINE_LOST_SPREAD_MIN) {
            /* === LINHA VIVEL: PID normal === */
            lost_counter = 0;

            /* Filtra erro suavemente */
            filtered_line_error = filtered_line_error * 0.5f + raw_error * 0.5f;

            /* SEMPRE atualiza direcao conhecida */
            last_valid_error = filtered_line_error;

            float abs_err = filtered_line_error;
            if (abs_err < 0.0f) {
                abs_err = -abs_err;
            }

            float correction = PID_Compute(&line_pid, 0.0f, filtered_line_error);

            /* Velocidade: boost na largada, desacelera em curva */
            float base = app_config.base_speed;
            uint32_t follow_elapsed = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
            if (follow_elapsed < FOLLOW_BOOST_MS) {
                base = FOLLOW_BOOST_SPEED;
            } else {
                float slowdown = abs_err / LINE_ERROR_FULL_SCALE;
                if (slowdown > 1.0f) {
                    slowdown = 1.0f;
                }
                base = app_config.base_speed - (app_config.base_speed - FOLLOW_MIN_SPEED) * LINE_CURVE_SLOWDOWN * slowdown;
            }

            cmd.vel_left = base + correction;
            cmd.vel_right = base - correction;

            if (cmd.vel_left < 0.0f) cmd.vel_left = 0.0f;
            if (cmd.vel_right < 0.0f) cmd.vel_right = 0.0f;
            if (cmd.vel_left > FOLLOW_MAX_SPEED) cmd.vel_left = FOLLOW_MAX_SPEED;
            if (cmd.vel_right > FOLLOW_MAX_SPEED) cmd.vel_right = FOLLOW_MAX_SPEED;
        } else {
            /* === LINHA PERDIDA: BUSCA LENTA === */
            lost_counter++;
            if (lost_counter > LINE_LOST_STOP_COUNT) {
                follower_state = STATE_STOPPING;
                continue;
            }

            /* NAO zera last_valid_error! Usa para saber direcao */
            /* Pivot lento: uma roda parada, outra gira devagar */
            if (last_valid_error > 0.02f) {
                /* Linha estava a direita: gira devagar para direita */
                cmd.vel_left = 0.0f;
                cmd.vel_right = LINE_SEARCH_POWER;
            } else if (last_valid_error < -0.02f) {
                /* Linha estava a esquerda: gira devagar para esquerda */
                cmd.vel_left = LINE_SEARCH_POWER;
                cmd.vel_right = 0.0f;
            } else {
                /* Sem direcao: gira para direita (default) */
                cmd.vel_left = 0.0f;
                cmd.vel_right = LINE_SEARCH_POWER;
            }

            /* Reset PID para nao acumular erro durante busca */
            PID_Reset(&line_pid);
        }

        App_ApplyMotorCmd(&cmd);

        if (now_ms - last_follow_tx >= FOLLOW_TX_INTERVAL_MS) {
            last_follow_tx = now_ms;
            char buf[128];
            int len = snprintf(buf, sizeof(buf),
                "F %s vl=%d vr=%d adj=%d spr=%u\r\n",
                (lost_counter > 0) ? "BUSCA" : "ON",
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

void App_MotorCtrlTask(void *argument)
{
    (void)argument;

    Encoder_Init();

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));

        Encoder_Update();

        MotorCmd_t cmd = g_motor_cmd;

        if (cmd.vel_left <= 0.0f && cmd.vel_right <= 0.0f) {
            Motor_Stop();
            continue;
        }

        Motor_SetPowerDiff(cmd.vel_left, cmd.vel_right);
    }
}
