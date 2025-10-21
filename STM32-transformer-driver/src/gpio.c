#include "gpio.h"

// Initialize GPIO for LED on PC13
void GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable GPIOC clock
  __HAL_RCC_GPIOC_CLK_ENABLE();

  // Configure GPIO pin for LED (PC13)
  // Set initial state to HIGH (LED off on Blue Pill, as it's active low)
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  // Configure pin as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
