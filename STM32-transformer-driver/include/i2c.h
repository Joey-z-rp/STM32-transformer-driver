#ifndef I2C_H
#define I2C_H

#include "stm32f1xx_hal.h"

// I2C Configuration
#define I2C_SCL_PIN GPIO_PIN_6
#define I2C_SDA_PIN GPIO_PIN_7
#define I2C_GPIO_PORT GPIOB
#define I2C_INSTANCE I2C1
#define I2C_CLOCK_SPEED 100000 // 100 kHz (standard mode)

// Function prototypes
void I2C_Init(void);
HAL_StatusTypeDef I2C_Write(uint8_t device_addr, uint8_t *data, uint16_t size);
I2C_HandleTypeDef *I2C_GetHandle(void);

#endif // I2C_H
