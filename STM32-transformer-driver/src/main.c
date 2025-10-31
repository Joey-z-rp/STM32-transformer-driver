#include "stm32f1xx_hal.h"
#include "clock.h"
#include "gpio.h"
#include "pwm.h"
#include "button.h"
#include "screen.h"
#include "temperature.h"
#include "current.h"
#include "fan.h"
#include <stdio.h>

int main(void)
{
  // Initialize HAL Library
  HAL_Init();

  // Configure system clock to 72MHz
  SystemClock_Config();

  // Initialize GPIO for LED
  GPIO_Init();

  // Initialize current sensor (ACS712 on PA6)
  Current_Init();

  // Calibrate current sensor at zero current (before PWM starts)
  Current_Calibrate();

  // Initialize PWM module (TIM2 CH1 on PA0)
  PWM_Init();

  // Initialize buttons (PA1-PA4)
  Button_Init();

  // Initialize LCD screen (I2C)
  Screen_Init();

  // Initialize temperature sensor (DS18B20 on PA5)
  Temperature_Init();

  // Initialize fan module (TIM3 CH2 on PA7)
  Fan_Init();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  // Display initial PWM values, temperature, and current
  Screen_DisplayAll(PWM_GetFrequency(), PWM_GetDutyCycle(), 0.0f, 0.0f);

  // Track previous values to detect changes
  uint32_t prev_frequency = PWM_GetFrequency();
  uint8_t prev_duty_cycle = PWM_GetDutyCycle();
  uint32_t sensor_update_counter = 0;

// Settings save delay (10 seconds = 10000 ms)
#define SETTINGS_SAVE_DELAY_MS 10000

  // Main loop
  while (1)
  {
    // Process button inputs
    Button_Process();

    // Check if settings need to be saved (after 10 seconds of inactivity)
    if (PWM_AreSettingsDirty())
    {
      uint32_t current_time = HAL_GetTick();
      uint32_t dirty_time = PWM_GetDirtyTimestamp();

      // Handle timer overflow (wraps around every ~49 days)
      uint32_t elapsed_time;
      if (current_time >= dirty_time)
      {
        elapsed_time = current_time - dirty_time;
      }
      else
      {
        // Timer wrapped around
        elapsed_time = (UINT32_MAX - dirty_time) + current_time + 1;
      }

      if (elapsed_time >= SETTINGS_SAVE_DELAY_MS)
      {
        PWM_SaveSettings();
      }
    }

    // Get current PWM values
    uint32_t current_frequency = PWM_GetFrequency();
    uint8_t current_duty_cycle = PWM_GetDutyCycle();

    // Update screen only for changed values (with new format)
    if (current_frequency != prev_frequency || current_duty_cycle != prev_duty_cycle)
    {
      // Update line 1: PWM frequency and duty cycle
      char buffer[32];
      Screen_SetCursor(0, 0);
      if (current_frequency >= 1000)
      {
        // Display in kHz with 1 decimal place
        uint32_t khz_int = current_frequency / 1000;
        uint32_t khz_frac = (current_frequency % 1000) / 100; // Get first decimal digit
        snprintf(buffer, sizeof(buffer), "PWM %lu.%lukHz %u%%  ", khz_int, khz_frac, current_duty_cycle);
      }
      else
      {
        // Display in Hz with 1 decimal place
        uint32_t hz_int = current_frequency / 10;
        uint32_t hz_frac = current_frequency % 10;
        snprintf(buffer, sizeof(buffer), "PWM %lu.%luHz %u%%  ", hz_int, hz_frac, current_duty_cycle);
      }
      Screen_Print(buffer);

      prev_frequency = current_frequency;
      prev_duty_cycle = current_duty_cycle;
    }

    // Read and display temperature and current every ~1 second (100 iterations * 10ms)
    sensor_update_counter++;
    if (sensor_update_counter >= 100)
    {
      sensor_update_counter = 0;

      // Read temperature from DS18B20
      float temperature = Temperature_Read();

      // Check temperature and enforce thermal cutoff if needed
      PWM_ThermalCheck(temperature);

      // Update fan based on temperature
      Fan_Update(temperature);

      // Read current from ACS712
      float current = Current_Read();

      // Update line 2: temperature, current, and fan (new format)
      char buffer[32];
      Screen_SetCursor(1, 0);

      // Temperature display
      if (temperature > -999.0f) // Valid temperature reading
      {
        int16_t temp_int = (int16_t)temperature;
        int16_t temp_frac = (int16_t)((temperature - temp_int) * 10);
        if (temp_frac < 0)
          temp_frac = -temp_frac;
        snprintf(buffer, sizeof(buffer), "%d.%dC ", temp_int, temp_frac);
      }
      else
      {
        snprintf(buffer, sizeof(buffer), "ERR ");
      }
      Screen_Print(buffer);

      // Current display (2 decimal places)
      int16_t current_int = (int16_t)current;
      int16_t current_frac = (int16_t)((current - current_int) * 100);
      if (current_frac < 0)
        current_frac = -current_frac;
      snprintf(buffer, sizeof(buffer), "%d.%02dA ", current_int, current_frac);
      Screen_Print(buffer);

      // Fan display (show as percentage of operational range)
      uint8_t fan_output = Fan_GetOutputPercent();
      snprintf(buffer, sizeof(buffer), "F%u%% ", fan_output);
      Screen_Print(buffer);

      // Thermal cutoff indicator
      if (PWM_IsThermalCutoffActive())
      {
        snprintf(buffer, sizeof(buffer), "Overheat");
      }
      else
      {
        snprintf(buffer, sizeof(buffer), "        ");
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
