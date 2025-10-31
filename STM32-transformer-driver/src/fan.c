#include "fan.h"

// TIM3 handle for fan PWM generation
TIM_HandleTypeDef htim3_fan;

// Fan state variables
static uint8_t fan_duty_cycle = 0;    // Current fan duty cycle (0-100%)
static uint8_t fan_target_duty = 0;   // Target duty cycle for gradual ramping
static uint8_t fan_active = 0;        // Fan active flag
static uint32_t last_update_time = 0; // Last update timestamp

// System clock frequency (STM32F103C8 is 72MHz)
#define SYSTEM_CLOCK 72000000

/**
 * @brief Initialize fan PWM on TIM3 Channel 2 (PA7)
 *
 * Uses TIM3 CH2 on PA7 for fan PWM output
 * Fixed frequency at 25kHz (typical for PC fans)
 */
void Fan_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  // Enable clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  // Configure PA7 as TIM3_CH2 (alternate function push-pull)
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure TIM3 for 25kHz PWM (typical for fans)
  // PWM frequency = SYSTEM_CLOCK / ((Prescaler + 1) * (Period + 1))
  // 25000 Hz = 72000000 / ((0 + 1) * (2880 + 1))
  htim3_fan.Instance = TIM3;
  htim3_fan.Init.Prescaler = 0;
  htim3_fan.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3_fan.Init.Period = 2879; // (72000000 / 25000) - 1 = 2879
  htim3_fan.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3_fan.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  HAL_TIM_PWM_Init(&htim3_fan);

  // Configure PWM mode on channel 2
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0; // Start with fan off (0% duty cycle)
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3_fan, &sConfigOC, TIM_CHANNEL_2);

  // Start PWM generation
  HAL_TIM_PWM_Start(&htim3_fan, TIM_CHANNEL_2);

  // Initialize state
  fan_duty_cycle = 0;
  fan_target_duty = 0;
  fan_active = 0;
  last_update_time = HAL_GetTick();
}

/**
 * @brief Update fan speed based on temperature
 * @param temperature Current temperature in Celsius
 *
 * Logic:
 * - Below threshold: Fan off (0%)
 * - Between threshold and max: Divided into FAN_SPEED_LEVELS discrete speeds
 * - At/above max: Maximum speed (FAN_MAX_DUTY)
 * - Gradual ramping for smooth transitions
 *
 * Example with FAN_SPEED_LEVELS=5, threshold=40°C, max=60°C, min=20%, max=60%:
 * - Below 40°C: Off (0%)
 * - 40-44°C: Speed 1 (20%)
 * - 44-48°C: Speed 2 (30%)
 * - 48-52°C: Speed 3 (40%)
 * - 52-56°C: Speed 4 (50%)
 * - 56-60°C: Speed 5 (60%)
 * - 60°C+: Speed 5 (60%)
 */
void Fan_Update(float temperature)
{
  uint32_t current_time = HAL_GetTick();

  // Automatic temperature-based control
  // Calculate target duty cycle based on temperature with discrete levels
  if (temperature < FAN_TEMP_THRESHOLD)
  {
    // Below threshold: turn fan off
    fan_target_duty = 0;
    fan_active = 0;
  }
  else if (temperature >= FAN_TEMP_MAX)
  {
    // At or above max temperature: maximum speed
    fan_target_duty = FAN_MAX_DUTY;
    fan_active = 1;
  }
  else
  {
    // Between threshold and max: use configurable discrete speed levels
    // Calculate which speed level based on temperature
    float temp_range = FAN_TEMP_MAX - FAN_TEMP_THRESHOLD;
    float temp_above_threshold = temperature - FAN_TEMP_THRESHOLD;
    float temp_step = temp_range / (float)FAN_SPEED_LEVELS;

    // Determine speed level (0 to FAN_SPEED_LEVELS-1)
    uint8_t speed_level = (uint8_t)(temp_above_threshold / temp_step);

    // Clamp to valid range
    if (speed_level >= FAN_SPEED_LEVELS)
    {
      speed_level = FAN_SPEED_LEVELS - 1;
    }

    // Calculate duty cycle for this speed level
    // Linearly distribute duty cycles from FAN_MIN_DUTY to FAN_MAX_DUTY
    uint8_t duty_range = FAN_MAX_DUTY - FAN_MIN_DUTY;
    fan_target_duty = FAN_MIN_DUTY + (duty_range * speed_level) / (FAN_SPEED_LEVELS - 1);

    fan_active = 1;
  }

  // Gradual ramping: update duty cycle towards target
  // Only update at specified interval for smooth ramping
  if ((current_time - last_update_time) >= FAN_UPDATE_INTERVAL)
  {
    last_update_time = current_time;

    if (fan_duty_cycle < fan_target_duty)
    {
      // Ramp up
      fan_duty_cycle += FAN_RAMP_STEP;
      if (fan_duty_cycle > fan_target_duty)
        fan_duty_cycle = fan_target_duty;
    }
    else if (fan_duty_cycle > fan_target_duty)
    {
      // Ramp down
      if (fan_duty_cycle >= FAN_RAMP_STEP)
        fan_duty_cycle -= FAN_RAMP_STEP;
      else
        fan_duty_cycle = 0;

      if (fan_duty_cycle < fan_target_duty)
        fan_duty_cycle = fan_target_duty;
    }

    // Update PWM duty cycle
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3_fan);
    uint32_t pulse = (period + 1) * fan_duty_cycle / 100;
    __HAL_TIM_SET_COMPARE(&htim3_fan, TIM_CHANNEL_2, pulse);
  }
}

/**
 * @brief Get current fan duty cycle
 * @return Current fan duty cycle in percentage (0-100)
 */
uint8_t Fan_GetDutyCycle(void)
{
  return fan_duty_cycle;
}

/**
 * @brief Get fan output as percentage of operational range
 * @return Fan output percentage (0-100%)
 *
 * Returns fan speed as a percentage of its operational range:
 * - 0% = Fan off
 * - 100% = Fan at maximum configured speed (FAN_MAX_DUTY)
 * This provides accurate display regardless of FAN_MAX_DUTY setting
 */
uint8_t Fan_GetOutputPercent(void)
{
  if (fan_duty_cycle == 0)
  {
    return 0;
  }

  // Calculate percentage based on operational range (MIN to MAX duty)
  if (fan_duty_cycle < FAN_MIN_DUTY)
  {
    return 0;
  }

  // Map duty cycle from [FAN_MIN_DUTY, FAN_MAX_DUTY] to [0, 100]
  uint8_t duty_range = FAN_MAX_DUTY - FAN_MIN_DUTY;
  uint8_t duty_above_min = fan_duty_cycle - FAN_MIN_DUTY;
  uint8_t output_percent = (duty_above_min * 100) / duty_range;

  // Clamp to 100%
  if (output_percent > 100)
    output_percent = 100;

  return output_percent;
}

/**
 * @brief Check if fan is active
 * @return 1 if fan is active, 0 if off
 */
uint8_t Fan_IsActive(void)
{
  return fan_active;
}
