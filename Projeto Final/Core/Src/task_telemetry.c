/**
 * @file task_telemetry.c
 * @brief Envia CSV de status a 5Hz pela LPUART (debug ST-LINK VCP) e Bluetooth
 *        e drena o ringbuffer RX do BT processando comandos.
 *
 * Formato CSV:  state,base_speed,dist_total_cm,x,y,theta_deg,obstacle_cm
 *
 * Adicionado em feature/joao por João Santos.
 */
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "app_tasks.h"
#include "bluetooth.h"
#include "odometry.h"
#include "ultrasonic.h"
#include "main.h"

extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart3;

#define TELEMETRY_PERIOD_MS  200U  /* 5Hz */

void App_TelemetryTask(void *argument)
{
    (void)argument;
    TickType_t next_wake = xTaskGetTickCount();
    char buf[96];

    for (;;) {
        int n = snprintf(buf, sizeof(buf),
                         "%d,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n",
                         (int)follower_state,
                         (double)app_config.base_speed,
                         (double)Odometry_GetDistanceCm(),
                         (double)Odometry_GetX(),
                         (double)Odometry_GetY(),
                         (double)Odometry_GetThetaDeg(),
                         (double)Ultrasonic_GetDistanceCm());
        if (n > 0) {
            HAL_UART_Transmit(&hlpuart1, (uint8_t *)buf, (uint16_t)n, 50U);
            HAL_UART_Transmit(&huart3,   (uint8_t *)buf, (uint16_t)n, 50U);
        }

        /* Processa comandos recebidos pelo Bluetooth. */
        Bluetooth_Poll();

        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}
