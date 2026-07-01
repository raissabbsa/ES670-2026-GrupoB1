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
#include <ctype.h>

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t s_mon_auto = 0U;

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

static void BT_Trim(char *s)
{
    char *start = s;

    if (s == NULL) {
        return;
    }

    while (*s != '\0') {
        *s = (char)toupper((unsigned char)*s);
        s++;
    }

    while (s > start) {
        s--;
        if (*s >= 'A' && *s <= 'Z') {
            break;
        }
        if (*s >= '0' && *s <= '9') {
            break;
        }
        *s = '\0';
    }
}

static void BT_ProcessLine(char *line)
{
    /* Echo em HEX de toda linha recebida. Util para diagnosticar
     * terminais que mandam caracteres invisíveis ou HC-05 corrompendo. */
    {
        char hex_buf[80];
        int n = 0;
        int llen = (int)strlen(line);
        int show = (llen < 16) ? llen : 16;
        for (int i = 0; i < show; i++) {
            n += snprintf(hex_buf + n, sizeof(hex_buf) - n,
                          "%02X ", (uint8_t)line[i]);
        }
        char echo[96];
        int elen = snprintf(echo, sizeof(echo), "$RAW[%d]=%s\r\n", llen, hex_buf);
        if (elen > 0 && elen < (int)sizeof(echo)) {
            BT_Send(echo);
        }
    }

    if (strcmp(line, "$HELP") == 0) {
        BT_Send("$HELP,GET|PID|MOT|MON|ALL,SET|KP|KI|KD|SPD|LSCL|RSCL|MON,CMD|START|STOP|FWD|REV|LEFT|RIGHT|JOY,RAW\r\n");
        return;
    }

    /* Aceita tanto "$GET,PID" quanto "$GETPID" (alguns terminais removem a virgula). */
    const char *get_param = NULL;
    if (strncmp(line, "$GET,", 5) == 0) {
        get_param = line + 5;
    } else if (strncmp(line, "$GET", 4) == 0 && line[4] != ',') {
        get_param = line + 4;
    }

    if (get_param != NULL) {
        char param[32];
        strncpy(param, get_param, sizeof(param) - 1);
        param[sizeof(param) - 1] = '\0';
        BT_Trim(param);

        if (strcmp(param, "PID") == 0) {
            float kp, ki, kd, spd;
            char buf[64];
            App_GetLinePidConfig(&kp, &ki, &kd, &spd);
            int len = snprintf(buf, sizeof(buf),
                "$PID,%.3f,%.3f,%.3f,%.3f\r\n", kp, ki, kd, spd);
            if (len > 0 && len < (int)sizeof(buf)) {
                BT_Send(buf);
            }
        } else if (strcmp(param, "MOT") == 0) {
            float ls, rs;
            char buf[48];
            Motor_GetScales(&ls, &rs);
            int len = snprintf(buf, sizeof(buf),
                "$MOT,%.3f,%.3f\r\n", ls, rs);
            if (len > 0 && len < (int)sizeof(buf)) {
                BT_Send(buf);
            }
        } else if (strcmp(param, "MON") == 0) {
            Telemetry_t telem;
            char buf[80];
            Telemetry_Get(&telem);
            int len = snprintf(buf, sizeof(buf),
                "$MON,%d,%d,%d,%d,%u,%u\r\n",
                (int)telem.distance_cm,
                (int)telem.speed_cms,
                (int)telem.heading_deg,
                (int)telem.battery_pct,
                (unsigned)telem.obstacle_cm,
                (unsigned)telem.state);
            if (len > 0 && len < (int)sizeof(buf)) {
                BT_Send(buf);
            }
        } else if (strcmp(param, "ALL") == 0) {
            /* Retorna KP, KI, KD, SPD, LSCL, RSCL, center target, escala dos motores. */
            float kp, ki, kd, spd;
            float ls, rs;
            App_GetLinePidConfig(&kp, &ki, &kd, &spd);
            Motor_GetScales(&ls, &rs);
            char buf[96];
            int len = snprintf(buf, sizeof(buf),
                "$ALL,KP=%.3f,KI=%.3f,KD=%.3f,SPD=%.3f,LSCL=%.3f,RSCL=%.3f\r\n",
                kp, ki, kd, spd, ls, rs);
            if (len > 0 && len < (int)sizeof(buf)) {
                BT_Send(buf);
            }
        } else {
            BT_Send("$ERR,GET\r\n");
        }
        return;
    }

    if (strncmp(line, "$SET,", 5) == 0) {
        char *param = line + 5;
        BT_Trim(param);

        if (strcmp(param, "PID") == 0) {
            float kp, ki, kd, spd;
            char buf[64];
            App_GetLinePidConfig(&kp, &ki, &kd, &spd);
            int len = snprintf(buf, sizeof(buf),
                "$PID,%.3f,%.3f,%.3f,%.3f\r\n", kp, ki, kd, spd);
            if (len > 0 && len < (int)sizeof(buf)) {
                BT_Send(buf);
            }
        } else if (strcmp(param, "MOT") == 0) {
            float ls, rs;
            char buf[48];
            Motor_GetScales(&ls, &rs);
            int len = snprintf(buf, sizeof(buf),
                "$MOT,%.3f,%.3f\r\n", ls, rs);
            if (len > 0 && len < (int)sizeof(buf)) {
                BT_Send(buf);
            }
        } else if (strcmp(param, "MON") == 0) {
            Telemetry_t telem;
            char buf[80];
            Telemetry_Get(&telem);
            int len = snprintf(buf, sizeof(buf),
                "$MON,%d,%d,%d,%d,%u,%u\r\n",
                (int)telem.distance_cm,
                (int)telem.speed_cms,
                (int)telem.heading_deg,
                (int)telem.battery_pct,
                (unsigned)telem.obstacle_cm,
                (unsigned)telem.state);
            if (len > 0 && len < (int)sizeof(buf)) {
                BT_Send(buf);
            }
        } else {
            char echo_buf[96];
            int echo_len = snprintf(echo_buf, sizeof(echo_buf), "$ECHO,%s\r\n", line);
            if (echo_len > 0 && echo_len < (int)sizeof(echo_buf)) {
                BT_Send(echo_buf);
            }
            BT_Send("$ERR,GET\r\n");
        }
        return;
    }

    if (strncmp(line, "$SET,", 5) == 0) {
        char *param = line + 5;
        char *comma = strchr(param, ',');
        if (!comma) {
            BT_Send("$ERR\r\n");
            return;
        }
        *comma = '\0';
        BT_Trim(param);
        float val = (float)atof(comma + 1);
        uint8_t ok = 0U;

        if (strcmp(param, "KP") == 0) {
            App_SetLinePidGains(val, app_config.line_Ki, app_config.line_Kd);
            ok = 1U;
        } else if (strcmp(param, "KI") == 0) {
            App_SetLinePidGains(app_config.line_Kp, val, app_config.line_Kd);
            ok = 1U;
        } else if (strcmp(param, "KD") == 0) {
            App_SetLinePidGains(app_config.line_Kp, app_config.line_Ki, val);
            ok = 1U;
        } else if (strcmp(param, "SPD") == 0) {
            App_SetBaseSpeed(val);
            ok = 1U;
        } else if (strcmp(param, "LSCL") == 0) {
            Motor_SetLeftScale(val);
            ok = 1U;
        } else if (strcmp(param, "RSCL") == 0) {
            Motor_SetRightScale(val);
            ok = 1U;
        } else if (strcmp(param, "MON") == 0) {
            s_mon_auto = (val >= 0.5f) ? 1U : 0U;
            ok = 1U;
        }

        BT_Send(ok ? "$OK\r\n" : "$ERR\r\n");
        return;
    }

    if (strncmp(line, "$CMD,", 5) == 0) {
        char *cmd = line + 5;
        MotorCmd_t mcmd = {0.0f, 0.0f};
        uint8_t ok = 0U;
        BT_Trim(cmd);

        if (strcmp(cmd, "FWD") == 0) {
            follower_state = STATE_MANUAL;
            mcmd.vel_left = 0.50f;
            mcmd.vel_right = 0.50f;
            ok = 1U;
        } else if (strcmp(cmd, "REV") == 0) {
            follower_state = STATE_MANUAL;
            mcmd.vel_left = -0.50f;
            mcmd.vel_right = -0.50f;
            ok = 1U;
        } else if (strcmp(cmd, "LEFT") == 0) {
            follower_state = STATE_MANUAL;
            mcmd.vel_left = 0.0f;
            mcmd.vel_right = 0.50f;
            ok = 1U;
        } else if (strcmp(cmd, "RIGHT") == 0) {
            follower_state = STATE_MANUAL;
            mcmd.vel_left = 0.50f;
            mcmd.vel_right = 0.0f;
            ok = 1U;
        } else if (strcmp(cmd, "STOP") == 0) {
            follower_state = STATE_IDLE;
            ok = 1U;
        } else if (strcmp(cmd, "START") == 0) {
            follower_state = STATE_ALIGNING;
            ok = 1U;
        }

        if (ok) {
            g_motor_cmd = mcmd;
            BT_Send("$OK\r\n");
        } else {
            BT_Send("$ERR\r\n");
        }
        return;
    }

    BT_Send("$ERR,CMD\r\n");
}

void App_BluetoothTask(void *argument)
{
    (void)argument;
    extern UART_HandleTypeDef huart3;

    Bluetooth_Init(&huart3);
    BT_Send("$BT,ready,9600\r\n");

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

        if (s_mon_auto &&
            (xTaskGetTickCount() - last_send) >= pdMS_TO_TICKS(1000)) {
            last_send = xTaskGetTickCount();
            Telemetry_Get(&telem);
            int len = snprintf(tx_buf, sizeof(tx_buf),
                "$MON,%d,%d,%d,%d,%u,%u\r\n",
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
