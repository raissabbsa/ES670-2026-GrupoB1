/**
 * @file bluetooth.h
 * @brief Wrapper sobre USART3 (HC-05) para controle remoto e sintonia.
 *
 * Protocolo ASCII linha-a-linha terminada em '\n'. Comandos suportados:
 *
 *   PING               -> resposta "PONG"
 *   GO                 -> avança FSM (equivalente a ENTER)
 *   STOP               -> volta para IDLE
 *   STATE?             -> resposta "STATE:<n>"
 *   VEL=<float>        -> ajusta base_speed
 *   KP_L=<float>       -> ajusta line_Kp
 *   KI_L=<float>       -> ajusta line_Ki
 *   KD_L=<float>       -> ajusta line_Kd
 *   KP_V=<float>       -> ajusta speed_Kp
 *   KI_V=<float>       -> ajusta speed_Ki
 *   KD_V=<float>       -> ajusta speed_Kd
 *
 * Adicionado em feature/joao por João Santos.
 */
#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "main.h"

void Bluetooth_Init(void);
void Bluetooth_Poll(void);                            /* drena RX, processa comandos */
void Bluetooth_SendLine(const char *line);            /* helper p/ telemetria */

/* Chamado pelo HAL_UART_RxCpltCallback quando o byte chega — leve em ISR. */
void Bluetooth_RxIsr(void);

#endif /* BLUETOOTH_H */
