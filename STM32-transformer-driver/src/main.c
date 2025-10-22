#include "stm32f1xx_hal.h"
#include "clock.h"
#include "gpio.h"
#include "pwm.h"
#include "button.h"
#include "screen.h"
#include "temperature.h"
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

  // Initialize temperature sensor (DS18B20 on PA5)
  Temperature_Init();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  // Display initial PWM values and temperature
  Screen_DisplayPWMandTemp(PWM_GetFrequency(), PWM_GetDutyCycle(), 0.0f);

  // Track previous values to detect changes
  uint32_t prev_frequency = PWM_GetFrequency();
  uint8_t prev_duty_cycle = PWM_GetDutyCycle();
  uint32_t temp_update_counter = 0;

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
      char buffer[32];
      Screen_SetCursor(0, 0);
      if (current_frequency >= 1000)
      {
        // Display in kHz with 1 decimal place
        uint32_t khz_int = current_frequency / 1000;
        uint32_t khz_frac = (current_frequency % 1000) / 100; // Get first decimal digit
        snprintf(buffer, sizeof(buffer), "F:%lu.%lukHz D:%u%%  ", khz_int, khz_frac, current_duty_cycle);
      }
      else
      {
        // Display in Hz with 1 decimal place
        uint32_t hz_int = current_frequency / 10;
        uint32_t hz_frac = current_frequency % 10;
        snprintf(buffer, sizeof(buffer), "F:%lu.%luHz D:%u%%  ", hz_int, hz_frac, current_duty_cycle);
      }
      Screen_Print(buffer);

      prev_frequency = current_frequency;
      prev_duty_cycle = current_duty_cycle;
    }

    // Read and display temperature every ~2 seconds (200 iterations * 10ms)
    temp_update_counter++;
    if (temp_update_counter >= 200)
    {
      temp_update_counter = 0;

      // Read temperature from DS18B20
      float temperature = Temperature_Read();

      // Update line 2 with temperature
      char buffer[32];
      Screen_SetCursor(1, 0);
      if (temperature > -999.0f) // Valid temperature reading
      {
        // Manually format temperature (avoid floating-point printf)
        int16_t temp_int = (int16_t)temperature;
        int16_t temp_frac = (int16_t)((temperature - temp_int) * 10);
        if (temp_frac < 0)
          temp_frac = -temp_frac; // Handle negative temps
        snprintf(buffer, sizeof(buffer), "Temp: %d.%dC    ", temp_int, temp_frac);
      }
      else
      {
        snprintf(buffer, sizeof(buffer), "Temp: Error    ");
      }
      Screen_Print(buffer);
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
