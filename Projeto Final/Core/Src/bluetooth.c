/**
 * @file bluetooth.c
 * @brief Adicionado em feature/joao por João Santos.
 *
 * Implementação minimalista:
 *   - HAL_UART_Receive_IT() recebe 1 byte por vez
 *   - byte é enfileirado num ringbuffer
 *   - Bluetooth_Poll() (chamado pela TelemetryTask) drena e parseia
 *
 * Para os parâmetros do PID e velocidade, alteramos diretamente o struct
 * `app_config` declarado em app_tasks.h.
 */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "bluetooth.h"
#include "app_tasks.h"

extern UART_HandleTypeDef huart3;

#define BT_RING_SIZE  128U

static volatile uint8_t  g_rx_byte;
static volatile uint8_t  g_ring[BT_RING_SIZE];
static volatile uint16_t g_head;
static volatile uint16_t g_tail;

void Bluetooth_Init(void)
{
    g_head = 0U;
    g_tail = 0U;
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&g_rx_byte, 1U);
}

void Bluetooth_RxIsr(void)
{
    uint16_t next = (uint16_t)((g_head + 1U) % BT_RING_SIZE);
    if (next != g_tail) {
        g_ring[g_head] = g_rx_byte;
        g_head = next;
    }
    /* Rearma para o próximo byte. */
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&g_rx_byte, 1U);
}

void Bluetooth_SendLine(const char *line)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)line, (uint16_t)strlen(line), 50U);
    HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", 2U, 10U);
}

static int ring_pop(void)
{
    if (g_head == g_tail) return -1;
    int c = g_ring[g_tail];
    g_tail = (uint16_t)((g_tail + 1U) % BT_RING_SIZE);
    return c;
}

static void trim(char *s)
{
    size_t n = strlen(s);
    while (n > 0 && (s[n-1] == '\r' || s[n-1] == ' ' || s[n-1] == '\t')) {
        s[--n] = '\0';
    }
}

static void handle_line(char *line)
{
    trim(line);
    if (line[0] == '\0') return;

    if (strcmp(line, "PING") == 0) { Bluetooth_SendLine("PONG"); return; }

    if (strcmp(line, "GO") == 0) {
        if (follower_state == STATE_IDLE  || follower_state == STATE_STOPPED ||
            follower_state == STATE_DEBUG) {
            follower_state = STATE_ALIGNING;
        }
        Bluetooth_SendLine("OK");
        return;
    }
    if (strcmp(line, "STOP") == 0) {
        if (follower_state == STATE_FOLLOWING || follower_state == STATE_IN_CROSSING) {
            follower_state = STATE_STOPPING;
        } else {
            follower_state = STATE_IDLE;
        }
        Bluetooth_SendLine("OK");
        return;
    }
    if (strcmp(line, "STATE?") == 0) {
        char buf[16];
        snprintf(buf, sizeof(buf), "STATE:%d", (int)follower_state);
        Bluetooth_SendLine(buf);
        return;
    }

    /* Comandos param=value */
    char *eq = strchr(line, '=');
    if (!eq) { Bluetooth_SendLine("ERR"); return; }
    *eq = '\0';
    float val = strtof(eq + 1, NULL);

    if      (strcmp(line, "VEL")  == 0) app_config.base_speed = val;
    else if (strcmp(line, "KP_L") == 0) app_config.line_Kp    = val;
    else if (strcmp(line, "KI_L") == 0) app_config.line_Ki    = val;
    else if (strcmp(line, "KD_L") == 0) app_config.line_Kd    = val;
    else if (strcmp(line, "KP_V") == 0) app_config.speed_Kp   = val;
    else if (strcmp(line, "KI_V") == 0) app_config.speed_Ki   = val;
    else if (strcmp(line, "KD_V") == 0) app_config.speed_Kd   = val;
    else { Bluetooth_SendLine("ERR"); return; }

    Bluetooth_SendLine("OK");
}

void Bluetooth_Poll(void)
{
    static char   line[64];
    static uint8_t pos = 0U;
    int c;

    while ((c = ring_pop()) >= 0) {
        if (c == '\n') {
            line[pos] = '\0';
            handle_line(line);
            pos = 0U;
        } else if (pos < (uint8_t)(sizeof(line) - 1U)) {
            line[pos++] = (char)c;
        } else {
            pos = 0U;
        }
    }
}
