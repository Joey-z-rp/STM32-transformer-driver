#include "stm32f1xx_hal.h"
#include "gpio.h"

int main(void)
{
  // Initialize HAL Library
  HAL_Init();

  // Initialize GPIO for LED
  GPIO_Init();

  // Main loop - blink LED
  while (1)
  {
    // Toggle LED on PC13
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    // Delay 500ms
    HAL_Delay(500);
  }
}

// SysTick interrupt handler (required for HAL_Delay)
void SysTick_Handler(void)
{
  HAL_IncTick();
}
