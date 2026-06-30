/**
 * @file task_display.c
 * @brief LCD 16x2 (I2C2 @ 0x27) atualizado a 2 Hz.
 *
 * Layout:
 *   Linha 0: "S:<state> V:<vel>"
 *   Linha 1: "D:<dist>cm O:<obs>"
 *
 * Adicionado em feature/joao por João Santos.
 */
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "app_tasks.h"
#include "lcd_hd44780_i2c.h"
#include "ultrasonic.h"
#include "odometry.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c2;

#define DISPLAY_PERIOD_MS  500U
#define DISPLAY_ADDR       0x27U
#define DISPLAY_LINES      2U
#define DISPLAY_COLS       16U

static const char *state_short_name(LineFollower_State s)
{
    switch (s) {
        case STATE_IDLE:           return "IDL";
        case STATE_ALIGNING:       return "ALI";
        case STATE_CALIBRATING:    return "CAL";
        case STATE_FOLLOWING:      return "RUN";
        case STATE_IN_CROSSING:    return "CRS";
        case STATE_STOPPING:       return "STP";
        case STATE_STOPPED:        return "FIN";
        case STATE_DEBUG:          return "DBG";
        case STATE_STOP_OBSTACLE:  return "OBS";
        case STATE_EMERGENCY:      return "EMG";
        default:                   return "???";
    }
}

void App_DisplayTask(void *argument)
{
    (void)argument;
    uint8_t line0[17];
    uint8_t line1[17];

    /* Init blocking — só após o scheduler subir e os outros drivers estarem ok. */
    lcdInit(&hi2c2, (uint8_t)DISPLAY_ADDR, (uint8_t)DISPLAY_LINES, (uint8_t)DISPLAY_COLS);
    lcdBacklightOn();
    lcdDisplayClear();

    TickType_t next_wake = xTaskGetTickCount();

    for (;;) {
        float vel  = (float)app_config.base_speed;
        float dist = Odometry_GetDistanceCm();
        float obs  = Ultrasonic_GetDistanceCm();

        snprintf((char *)line0, sizeof(line0),
                 "S:%s V:%4d", state_short_name(follower_state), (int)(vel * 100));
        snprintf((char *)line1, sizeof(line1),
                 "D:%4dcm O:%3d", (int)dist, (int)obs);

        lcdSetCursorPosition(0, 0);
        lcdPrintStr(line0, (uint8_t)strlen((char *)line0));
        lcdSetCursorPosition(0, 1);
        lcdPrintStr(line1, (uint8_t)strlen((char *)line1));

        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    }
}
