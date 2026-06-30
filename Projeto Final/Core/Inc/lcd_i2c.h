#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

#define LCD_ADDR       0x27
#define LCD_ADDR_ALT   0x3F
#define LCD_COLS       16
#define LCD_ROWS       2

void LCD_Init(I2C_HandleTypeDef *hi2c);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(const char *str);
void LCD_PrintAt(uint8_t row, uint8_t col, const char *str);

void App_LcdTask(void *argument);

#endif
