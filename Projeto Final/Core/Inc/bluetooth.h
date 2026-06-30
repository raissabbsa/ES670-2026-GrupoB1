#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

#define BT_RX_BUF_SIZE  128

void Bluetooth_Init(UART_HandleTypeDef *huart);
void Bluetooth_RxCallback(uint8_t byte);

void App_BluetoothTask(void *argument);

#endif
