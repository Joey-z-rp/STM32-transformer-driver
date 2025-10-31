#include "fan.h"

// TIM3 handle for fan PWM generation
TIM_HandleTypeDef htim3_fan;

// Fan state variables
static uint8_t fan_duty_cycle = 0;    // Current fan duty cycle (0-100%)
static uint8_t fan_active = 0;        // Fan active flag
static uint8_t fan_manual_mode = 0;   // Manual mode flag (1 = manual, 0 = auto)
static uint8_t fan_manual_level = 0;  // Current manual speed level (0 = off, 1-FAN_SPEED_LEVELS = speed levels)
static uint8_t fan_current_level = 0; // Current speed level (0 = off, 1-FAN_SPEED_LEVELS = speed levels)

// Temperature threshold check variables
static uint32_t fan_above_threshold_start_time = 0; // Time when temperature first went above threshold
static uint8_t fan_above_threshold_enabled = 0;     // Flag indicating temp has been above threshold for 5s

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
  fan_active = 0;
  fan_manual_mode = 0;
  fan_manual_level = 0;
  fan_current_level = 0;

  // Initialize temperature threshold check
  fan_above_threshold_start_time = 0;
  fan_above_threshold_enabled = 0;
}

static void Fan_SetDutyCycle(uint8_t duty_cycle)
{
  fan_duty_cycle = duty_cycle;

  // Update PWM duty cycle immediately
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3_fan);
  uint32_t pulse = (period + 1) * fan_duty_cycle / 100;
  __HAL_TIM_SET_COMPARE(&htim3_fan, TIM_CHANNEL_2, pulse);
}

void Fan_CycleSpeedLevel(void)
{
  fan_manual_mode = 1;

  fan_manual_level++;
  if (fan_manual_level > FAN_SPEED_LEVELS)
  {
    fan_manual_level = 0;
  }

  if (fan_manual_level == 0)
  {
    Fan_SetDutyCycle(0);
    fan_active = 0;
    fan_current_level = 0;
  }
  else
  {
    uint8_t duty_range = FAN_MAX_DUTY - FAN_MIN_DUTY;
    uint8_t duty_cycle = FAN_MIN_DUTY + (duty_range * (fan_manual_level - 1)) / (FAN_SPEED_LEVELS - 1);
    Fan_SetDutyCycle(duty_cycle);
    fan_active = 1;
    fan_current_level = fan_manual_level;
  }
}

void Fan_Update(float temperature)
{
  if (fan_manual_mode)
  {
    // Manual mode: no temperature threshold check needed
    return;
  }

  // Temperature threshold check: wait 5 seconds above threshold before enabling fan
  // This prevents incorrect startup readings from affecting fan control
  uint32_t current_time = HAL_GetTick();

  // Check if temperature is above threshold
  if (temperature >= FAN_TEMP_THRESHOLD)
  {
    // Temperature is above threshold
    if (fan_above_threshold_start_time == 0)
    {
      // First time above threshold, start timer
      fan_above_threshold_start_time = current_time;
      fan_above_threshold_enabled = 0;
    }
    else
    {
      // Check if we've been above threshold for the required duration
      uint32_t duration_above_threshold;
      if (current_time >= fan_above_threshold_start_time)
      {
        duration_above_threshold = current_time - fan_above_threshold_start_time;
      }
      else
      {
        // Handle timer overflow
        duration_above_threshold = (UINT32_MAX - fan_above_threshold_start_time) + current_time + 1;
      }

      if (duration_above_threshold >= FAN_TEMP_ABOVE_THRESHOLD_DURATION_MS)
      {
        fan_above_threshold_enabled = 1;
      }
    }
  }
  else
  {
    // Temperature dropped below threshold, reset timer
    fan_above_threshold_start_time = 0;
    fan_above_threshold_enabled = 0;

    // Turn fan off if temperature is below threshold
    if (fan_duty_cycle != 0 || fan_current_level != 0)
    {
      Fan_SetDutyCycle(0);
      fan_current_level = 0;
      fan_active = 0;
    }
    return;
  }

  // Only update fan speed if temperature has been above threshold for 5 seconds
  if (!fan_above_threshold_enabled)
  {
    // Temperature above threshold but not for 5 seconds yet, keep fan off
    if (fan_duty_cycle != 0 || fan_current_level != 0)
    {
      Fan_SetDutyCycle(0);
      fan_current_level = 0;
      fan_active = 0;
    }
    return;
  }

  uint8_t target_duty;
  uint8_t target_level;

  // Temperature is above threshold and has been for 5+ seconds
  if (temperature >= FAN_TEMP_MAX)
  {
    // At or above max temperature: maximum speed (level FAN_SPEED_LEVELS)
    target_duty = FAN_MAX_DUTY;
    target_level = FAN_SPEED_LEVELS;
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

    // Convert 0-indexed level to 1-indexed (0 = off, 1-FAN_SPEED_LEVELS = speed levels)
    target_level = speed_level + 1;

    // Calculate duty cycle for this speed level
    // Linearly distribute duty cycles from FAN_MIN_DUTY to FAN_MAX_DUTY
    uint8_t duty_range = FAN_MAX_DUTY - FAN_MIN_DUTY;
    target_duty = FAN_MIN_DUTY + (duty_range * speed_level) / (FAN_SPEED_LEVELS - 1);

    fan_active = 1;
  }

  if (fan_duty_cycle != target_duty || fan_current_level != target_level)
  {
    Fan_SetDutyCycle(target_duty);
    fan_current_level = target_level;
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

uint8_t Fan_GetOutputPercent(void)
{
  if (fan_current_level == 0)
  {
    return 0;
  }

  uint8_t output_percent = (fan_current_level * 100) / FAN_SPEED_LEVELS;

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
