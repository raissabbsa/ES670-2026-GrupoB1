#include "bluetooth.h"
#include "telemetry.h"
#include "app_tasks.h"
#include "motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static UART_HandleTypeDef *s_huart = NULL;

static volatile uint8_t s_rx_buf[BT_RX_BUF_SIZE];
static volatile uint8_t s_rx_head = 0;
static volatile uint8_t s_rx_tail = 0;

void Bluetooth_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
}

void Bluetooth_RxCallback(uint8_t byte)
{
    uint8_t next = (s_rx_head + 1) % BT_RX_BUF_SIZE;
    if (next != s_rx_tail) {
        s_rx_buf[s_rx_head] = byte;
        s_rx_head = next;
    }
}

static uint8_t BT_Available(void)
{
    return s_rx_head != s_rx_tail;
}

static uint8_t BT_Read(void)
{
    uint8_t byte = s_rx_buf[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1) % BT_RX_BUF_SIZE;
    return byte;
}

static void BT_Send(const char *msg)
{
    if (s_huart == NULL || msg == NULL) return;
    HAL_UART_Transmit(s_huart, (uint8_t *)msg, (uint16_t)strlen(msg), 100);
}

static void BT_ProcessLine(char *line)
{
    if (strncmp(line, "$SET,", 5) == 0) {
        char *param = line + 5;
        char *comma = strchr(param, ',');
        if (!comma) return;
        *comma = '\0';
        float val = (float)atof(comma + 1);

        if (strcmp(param, "KP") == 0) {
            app_config.line_Kp = val;
        } else if (strcmp(param, "KI") == 0) {
            app_config.line_Ki = val;
        } else if (strcmp(param, "KD") == 0) {
            app_config.line_Kd = val;
        } else if (strcmp(param, "SPD") == 0) {
            app_config.base_speed = val;
        }
        BT_Send("$OK\n");
        return;
    }

    if (strncmp(line, "$CMD,", 5) == 0) {
        char *cmd = line + 5;
        MotorCmd_t mcmd = {0.0f, 0.0f};

        if (strcmp(cmd, "FWD") == 0) {
            follower_state = STATE_MANUAL;
            mcmd.vel_left = 0.35f;
            mcmd.vel_right = 0.35f;
        } else if (strcmp(cmd, "REV") == 0) {
            follower_state = STATE_MANUAL;
            mcmd.vel_left = -0.35f;
            mcmd.vel_right = -0.35f;
        } else if (strcmp(cmd, "LEFT") == 0) {
            follower_state = STATE_MANUAL;
            mcmd.vel_left = 0.0f;
            mcmd.vel_right = 0.35f;
        } else if (strcmp(cmd, "RIGHT") == 0) {
            follower_state = STATE_MANUAL;
            mcmd.vel_left = 0.35f;
            mcmd.vel_right = 0.0f;
        } else if (strcmp(cmd, "STOP") == 0) {
            follower_state = STATE_IDLE;
        } else if (strcmp(cmd, "START") == 0) {
            follower_state = STATE_ALIGNING;
        }
        g_motor_cmd = mcmd;
        return;
    }
}

void App_BluetoothTask(void *argument)
{
    (void)argument;
    extern UART_HandleTypeDef huart3;

    Bluetooth_Init(&huart3);

    char line_buf[80];
    uint8_t line_pos = 0;
    Telemetry_t telem;
    char tx_buf[120];

    TickType_t last_send = xTaskGetTickCount();

    for (;;) {
        osDelay(50);

        while (BT_Available()) {
            uint8_t c = BT_Read();
            if (c == '\n' || c == '\r') {
                if (line_pos > 0) {
                    line_buf[line_pos] = '\0';
                    BT_ProcessLine(line_buf);
                    line_pos = 0;
                }
            } else if (line_pos < sizeof(line_buf) - 1) {
                line_buf[line_pos++] = (char)c;
            }
        }

        if ((xTaskGetTickCount() - last_send) >= pdMS_TO_TICKS(1000)) {
            last_send = xTaskGetTickCount();
            Telemetry_Get(&telem);
            int len = snprintf(tx_buf, sizeof(tx_buf),
                "$MON,%d,%d,%d,%d,%u,%u\n",
                (int)telem.distance_cm,
                (int)telem.speed_cms,
                (int)telem.heading_deg,
                (int)telem.battery_pct,
                (unsigned)telem.obstacle_cm,
                (unsigned)telem.state);
            if (len > 0 && len < (int)sizeof(tx_buf)) {
                BT_Send(tx_buf);
            }
        }
    }
}
