#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include "stm32f1xx_hal.h"

/**
 * @file temperature.h
 * @brief DS18B20 1-Wire Temperature Sensor Driver
 *
 * This module provides functions to read temperature from a DS18B20 digital
 * temperature sensor using the 1-Wire protocol.
 *
 * Hardware Connection:
 * - Connect DS18B20 data line to PA5 (configurable via DS18B20_Pin)
 * - Connect DS18B20 VCC to 3.3V or 5V
 * - Connect DS18B20 GND to GND
 * - Add a 4.7K pull-up resistor between data line and VCC
 *
 * Usage:
 * 1. Call Temperature_Init() to initialize the module
 * 2. Call Temperature_Read() to get temperature in Celsius
 *    - Returns temperature value on success
 *    - Returns -1000.0f on error (sensor not detected)
 */

// DS18B20 Configuration
// Default pin: PA5 (can be changed by modifying DS18B20_GPIO_Port and DS18B20_Pin)
#define DS18B20_GPIO_Port GPIOA
#define DS18B20_Pin GPIO_PIN_5

// DS18B20 Commands
#define DS18B20_CMD_SKIPROM 0xCC
#define DS18B20_CMD_CONVERTTEMP 0x44
#define DS18B20_CMD_RSCRATCHPAD 0xBE

// Function prototypes
void Temperature_Init(void);
uint8_t Temperature_Start(void);
float Temperature_Read(void);
uint8_t Temperature_GetData(uint8_t *data);

#endif // TEMPERATURE_H
