#include "lcd_i2c.h"
#include "telemetry.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>

static I2C_HandleTypeDef *s_hi2c = NULL;
static uint8_t s_addr = LCD_ADDR << 1;
static uint8_t s_backlight = 0x08;

#define PIN_RS   0x01
#define PIN_EN   0x04

static void LCD_WriteI2C(uint8_t data)
{
    HAL_I2C_Master_Transmit(s_hi2c, s_addr, &data, 1, 10);
}

static void LCD_PulseEnable(uint8_t data)
{
    LCD_WriteI2C(data | PIN_EN | s_backlight);
    LCD_WriteI2C((data & ~PIN_EN) | s_backlight);
}

static void LCD_SendNibble(uint8_t nibble, uint8_t rs)
{
    uint8_t data = (nibble & 0xF0) | rs;
    LCD_PulseEnable(data);
}

static void LCD_SendByte(uint8_t byte, uint8_t rs)
{
    LCD_SendNibble(byte & 0xF0, rs);
    LCD_SendNibble((byte << 4) & 0xF0, rs);
}

static void LCD_Command(uint8_t cmd)
{
    LCD_SendByte(cmd, 0);
    if (cmd <= 0x03) {
        osDelay(2);
    }
}

void LCD_Init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;

    if (HAL_I2C_IsDeviceReady(hi2c, LCD_ADDR << 1, 2, 50) == HAL_OK) {
        s_addr = LCD_ADDR << 1;
    } else {
        s_addr = LCD_ADDR_ALT << 1;
    }

    osDelay(50);
    LCD_SendNibble(0x30, 0);
    osDelay(5);
    LCD_SendNibble(0x30, 0);
    osDelay(1);
    LCD_SendNibble(0x30, 0);
    LCD_SendNibble(0x20, 0);

    LCD_Command(0x28); /* 4-bit, 2 lines, 5x8 */
    LCD_Command(0x0C); /* display on, cursor off */
    LCD_Command(0x06); /* entry mode: increment */
    LCD_Command(0x01); /* clear */
    osDelay(2);
}

void LCD_Clear(void)
{
    LCD_Command(0x01);
    osDelay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t addr = col + (row == 0 ? 0x00 : 0x40);
    LCD_Command(0x80 | addr);
}

void LCD_Print(const char *str)
{
    while (*str) {
        LCD_SendByte((uint8_t)*str++, PIN_RS);
    }
}

void LCD_PrintAt(uint8_t row, uint8_t col, const char *str)
{
    LCD_SetCursor(row, col);
    LCD_Print(str);
}

void App_LcdTask(void *argument)
{
    (void)argument;
    extern I2C_HandleTypeDef hi2c2;

    LCD_Init(&hi2c2);
    LCD_PrintAt(0, 0, "Robo Seguidor");
    LCD_PrintAt(1, 0, "Aguardando...");

    char line[LCD_COLS + 1];
    Telemetry_t telem;

    for (;;) {
        osDelay(1000);
        Telemetry_Get(&telem);

        snprintf(line, sizeof(line), "D:%-4dcm V:%-3d ", (int)telem.distance_cm, (int)telem.speed_cms);
        LCD_PrintAt(0, 0, line);

        snprintf(line, sizeof(line), "H:%-4d Bat:%-3d%%", (int)telem.heading_deg, (int)telem.battery_pct);
        LCD_PrintAt(1, 0, line);
    }
}
