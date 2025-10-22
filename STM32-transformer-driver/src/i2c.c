#include "i2c.h"

// I2C handle
I2C_HandleTypeDef hi2c1;

/**
 * @brief Initialize I2C1 peripheral
 *        SCL: PB6, SDA: PB7
 */
void I2C_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable clocks
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  // Configure I2C GPIO pins (PB6=SCL, PB7=SDA)
  // Alternative function open-drain with pull-up
  GPIO_InitStruct.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

  // Configure I2C peripheral
  hi2c1.Instance = I2C_INSTANCE;
  hi2c1.Init.ClockSpeed = I2C_CLOCK_SPEED;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    // Initialization Error - could add error handling here
    while (1)
      ;
  }
}

/**
 * @brief Write data to I2C device
 * @param device_addr I2C device address (7-bit, will be left-shifted internally by HAL)
 * @param data Pointer to data buffer
 * @param size Number of bytes to write
 * @return HAL status
 */
HAL_StatusTypeDef I2C_Write(uint8_t device_addr, uint8_t *data, uint16_t size)
{
  return HAL_I2C_Master_Transmit(&hi2c1, device_addr << 1, data, size, HAL_MAX_DELAY);
}

/**
 * @brief Get I2C handle for advanced operations
 * @return Pointer to I2C handle
 */
I2C_HandleTypeDef *I2C_GetHandle(void)
{
  return &hi2c1;
}
