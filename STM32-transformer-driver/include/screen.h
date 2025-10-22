#ifndef SCREEN_H
#define SCREEN_H

#include "stm32f1xx_hal.h"

// LCD1602 I2C address (common addresses: 0x27, 0x3F)
#define LCD_I2C_ADDR 0x27

// LCD dimensions
#define LCD_ROWS 2
#define LCD_COLS 16

// Function prototypes
void Screen_Init(void);
void Screen_Clear(void);
void Screen_SetCursor(uint8_t row, uint8_t col);
void Screen_Print(const char *str);
void Screen_PrintNumber(uint32_t number);
void Screen_Backlight(uint8_t state);
void Screen_UpdatePWM(uint32_t frequency, uint8_t duty_cycle);
void Screen_UpdateFrequency(uint32_t frequency);
void Screen_UpdateDutyCycle(uint8_t duty_cycle);
void Screen_UpdateTemperature(float temperature);
void Screen_DisplayPWMandTemp(uint32_t frequency, uint8_t duty_cycle, float temperature);

#endif // SCREEN_H
