#include "stm32f1xx_hal.h"
#include "clock.h"
#include "gpio.h"
#include "pwm.h"
#include "button.h"
#include "screen.h"
#include <stdio.h>

int main(void)
{
  // Initialize HAL Library
  HAL_Init();

  // Configure system clock to 72MHz
  SystemClock_Config();

  // Initialize GPIO for LED
  GPIO_Init();

  // Initialize PWM module (TIM2 CH1 on PA0)
  PWM_Init();

  // Initialize buttons (PA1-PA4)
  Button_Init();

  // Initialize LCD screen (I2C)
  Screen_Init();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  // Display initial PWM values and temperature
  Screen_DisplayPWMandTemp(PWM_GetFrequency(), PWM_GetDutyCycle(), 0.0f);

  // Track previous values to detect changes
  uint32_t prev_frequency = PWM_GetFrequency();
  uint8_t prev_duty_cycle = PWM_GetDutyCycle();

  // Main loop
  while (1)
  {
    // Process button inputs
    Button_Process();

    // Get current PWM values
    uint32_t current_frequency = PWM_GetFrequency();
    uint8_t current_duty_cycle = PWM_GetDutyCycle();

    // Update screen only for changed values (with compact format)
    if (current_frequency != prev_frequency || current_duty_cycle != prev_duty_cycle)
    {
      // Update line 1 with frequency and duty cycle
      char buffer[20];
      Screen_SetCursor(0, 0);
      if (current_frequency >= 1000)
      {
        uint32_t khz = current_frequency / 1000;
        sprintf(buffer, "F:%lukHz D:%u%%    ", khz, current_duty_cycle);
      }
      else
      {
        sprintf(buffer, "F:%luHz D:%u%%    ", current_frequency, current_duty_cycle);
      }
      Screen_Print(buffer);

      prev_frequency = current_frequency;
      prev_duty_cycle = current_duty_cycle;
    }

    // Small delay
    HAL_Delay(10);
  }
}

// SysTick interrupt handler (required for HAL_Delay)
void SysTick_Handler(void)
{
  HAL_IncTick();
}
