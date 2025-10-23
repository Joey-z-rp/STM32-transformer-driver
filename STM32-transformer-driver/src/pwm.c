#include "pwm.h"

// TIM2 handle for PWM generation
TIM_HandleTypeDef htim2;

// Current PWM settings (defaults if no saved settings exist)
static uint32_t current_frequency = 10000; // Default: 10 kHz
static uint8_t current_duty_cycle = 0;     // Default: 0%

// System clock frequency (STM32F103C8 default is 72MHz)
#define SYSTEM_CLOCK 72000000

// Target minimum period for 1% duty cycle resolution (need at least 100 steps)
#define TARGET_MIN_PERIOD 100

// Flash storage settings
// STM32F103C8 has 64KB flash, using last page (1KB) for settings
#define FLASH_SETTINGS_PAGE_ADDR 0x0800F800 // Last 1KB page
#define FLASH_SETTINGS_MAGIC 0x50574D53     // "PWMS" magic number

// Settings structure stored in flash
typedef struct
{
  uint32_t magic;      // Magic number to verify valid data
  uint32_t frequency;  // Saved frequency
  uint8_t duty_cycle;  // Saved duty cycle
  uint8_t reserved[3]; // Padding for alignment
  uint32_t checksum;   // Simple checksum for data integrity
} PWM_Settings_t;

// Settings dirty tracking
static uint8_t settings_dirty = 0;
static uint32_t settings_dirty_timestamp = 0;

/**
 * @brief Calculate checksum for settings
 * @param settings Pointer to settings structure
 * @return Calculated checksum
 */
static uint32_t PWM_CalculateChecksum(const PWM_Settings_t *settings)
{
  uint32_t checksum = settings->magic;
  checksum ^= settings->frequency;
  checksum ^= (uint32_t)settings->duty_cycle;
  return checksum;
}

/**
 * @brief Load PWM settings from flash
 * @return 1 if settings loaded successfully, 0 otherwise
 */
static uint8_t PWM_LoadSettings(void)
{
  const PWM_Settings_t *flash_settings = (const PWM_Settings_t *)FLASH_SETTINGS_PAGE_ADDR;

  // Verify magic number
  if (flash_settings->magic != FLASH_SETTINGS_MAGIC)
  {
    return 0; // No valid settings found
  }

  // Verify checksum
  if (flash_settings->checksum != PWM_CalculateChecksum(flash_settings))
  {
    return 0; // Corrupted data
  }

  // Validate frequency range
  if (flash_settings->frequency < PWM_MIN_FREQ || flash_settings->frequency > PWM_MAX_FREQ)
  {
    return 0; // Invalid frequency
  }

  // Validate duty cycle range
  if (flash_settings->duty_cycle > 100)
  {
    return 0; // Invalid duty cycle
  }

  // Load settings
  current_frequency = flash_settings->frequency;
  current_duty_cycle = flash_settings->duty_cycle;

  return 1; // Settings loaded successfully
}

/**
 * @brief Save PWM settings to flash
 */
void PWM_SaveSettings(void)
{
  PWM_Settings_t settings;
  HAL_StatusTypeDef status;

  // Prepare settings structure
  settings.magic = FLASH_SETTINGS_MAGIC;
  settings.frequency = current_frequency;
  settings.duty_cycle = current_duty_cycle;
  settings.reserved[0] = 0;
  settings.reserved[1] = 0;
  settings.reserved[2] = 0;
  settings.checksum = PWM_CalculateChecksum(&settings);

  // Unlock flash
  HAL_FLASH_Unlock();

  // Erase the page
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PageError = 0;

  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = FLASH_SETTINGS_PAGE_ADDR;
  EraseInitStruct.NbPages = 1;

  status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

  if (status == HAL_OK)
  {
    // Write settings to flash (word by word)
    uint32_t *data = (uint32_t *)&settings;
    uint32_t address = FLASH_SETTINGS_PAGE_ADDR;
    uint32_t words = sizeof(PWM_Settings_t) / 4;

    for (uint32_t i = 0; i < words; i++)
    {
      status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data[i]);
      if (status != HAL_OK)
      {
        break;
      }
      address += 4;
    }
  }

  // Lock flash
  HAL_FLASH_Lock();

  // Clear dirty flag after successful save
  if (status == HAL_OK)
  {
    settings_dirty = 0;
    settings_dirty_timestamp = 0;
  }
}

/**
 * @brief Mark settings as dirty (changed by user)
 */
void PWM_MarkSettingsDirty(void)
{
  if (!settings_dirty)
  {
    settings_dirty = 1;
    settings_dirty_timestamp = HAL_GetTick();
  }
  else
  {
    // Update timestamp to reset the 10-second timer
    settings_dirty_timestamp = HAL_GetTick();
  }
}

/**
 * @brief Check if settings are dirty
 * @return 1 if settings are dirty, 0 otherwise
 */
uint8_t PWM_AreSettingsDirty(void)
{
  return settings_dirty;
}

/**
 * @brief Reset the dirty timer
 */
void PWM_ResetDirtyTimer(void)
{
  settings_dirty_timestamp = HAL_GetTick();
}

/**
 * @brief Get the timestamp when settings became dirty
 * @return Timestamp in milliseconds
 */
uint32_t PWM_GetDirtyTimestamp(void)
{
  return settings_dirty_timestamp;
}

/**
 * @brief Calculate optimal prescaler and period for given frequency
 * @param frequency Desired PWM frequency in Hz
 * @param prescaler Pointer to store calculated prescaler value
 * @param period Pointer to store calculated period value
 *
 * Strategy: Maximize the period (ARR) value to maintain good duty cycle resolution
 * PWM frequency = SYSTEM_CLOCK / ((Prescaler + 1) * (Period + 1))
 */
static void PWM_CalculateTimerParams(uint32_t frequency, uint32_t *prescaler, uint32_t *period)
{
  uint32_t timer_ticks = SYSTEM_CLOCK / frequency;

  // Try to keep period >= TARGET_MIN_PERIOD for good resolution
  // Start with prescaler = 0 and increase if needed
  *prescaler = 0;
  *period = timer_ticks - 1;

  // If period is too large (>65535 for 16-bit timer), increase prescaler
  while (*period > 65535)
  {
    (*prescaler)++;
    *period = (SYSTEM_CLOCK / ((*prescaler) + 1) / frequency) - 1;
  }

  // If period is too small (poor resolution), try to increase it
  // by adjusting prescaler, but ensure we don't exceed 16-bit limits
  if (*period < TARGET_MIN_PERIOD && *prescaler > 0)
  {
    // Try to find a better prescaler that gives period closer to TARGET_MIN_PERIOD
    uint32_t best_prescaler = *prescaler;
    uint32_t best_period = *period;
    uint32_t best_error = UINT32_MAX;

    for (uint32_t p = 0; p <= *prescaler; p++)
    {
      uint32_t test_period = (SYSTEM_CLOCK / ((p + 1) * frequency)) - 1;
      if (test_period <= 65535)
      {
        // Calculate actual frequency with this configuration
        uint32_t actual_freq = SYSTEM_CLOCK / ((p + 1) * (test_period + 1));
        uint32_t freq_error = (actual_freq > frequency) ? (actual_freq - frequency) : (frequency - actual_freq);

        // Prefer configurations with higher period if frequency error is acceptable
        if (test_period >= TARGET_MIN_PERIOD && freq_error < best_error)
        {
          best_prescaler = p;
          best_period = test_period;
          best_error = freq_error;
        }
      }
    }

    if (best_error < UINT32_MAX)
    {
      *prescaler = best_prescaler;
      *period = best_period;
    }
  }
}

/**
 * @brief Initialize PWM on TIM2 Channel 1 (PA0)
 */
void PWM_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  // Try to load saved settings from flash
  PWM_LoadSettings();

  // Enable clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();

  // Configure PA0 as TIM2_CH1 (alternate function push-pull)
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure TIM2 base
  htim2.Instance = TIM2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  // Calculate optimal prescaler and period for initial frequency
  // PWM frequency = SYSTEM_CLOCK / ((Prescaler + 1) * (Period + 1))
  // Strategy: Keep Period as high as possible for better duty cycle resolution
  uint32_t prescaler, period;
  PWM_CalculateTimerParams(current_frequency, &prescaler, &period);
  htim2.Init.Prescaler = prescaler;
  htim2.Init.Period = period;

  HAL_TIM_PWM_Init(&htim2);

  // Configure PWM mode on channel 1
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (htim2.Init.Period + 1) * current_duty_cycle / 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  // Start PWM generation
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

/**
 * @brief Set PWM frequency
 * @param frequency Desired frequency in Hz (1000 to 200000)
 */
void PWM_SetFrequency(uint32_t frequency)
{
  // Clamp frequency to valid range
  if (frequency < PWM_MIN_FREQ)
    frequency = PWM_MIN_FREQ;
  if (frequency > PWM_MAX_FREQ)
    frequency = PWM_MAX_FREQ;

  current_frequency = frequency;

  // Mark settings as dirty for delayed save
  PWM_MarkSettingsDirty();

  // Calculate optimal prescaler and period
  uint32_t prescaler, period;
  PWM_CalculateTimerParams(current_frequency, &prescaler, &period);

  // Update timer prescaler and period
  __HAL_TIM_SET_PRESCALER(&htim2, prescaler);
  __HAL_TIM_SET_AUTORELOAD(&htim2, period);

  // Update duty cycle pulse width to maintain percentage
  uint32_t pulse = (period + 1) * current_duty_cycle / 100;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}

/**
 * @brief Set PWM duty cycle
 * @param duty_cycle Desired duty cycle in percentage (0 to 100)
 */
void PWM_SetDutyCycle(uint8_t duty_cycle)
{
  // Clamp duty cycle to valid range
  if (duty_cycle > 100)
    duty_cycle = 100;

  current_duty_cycle = duty_cycle;

  // Mark settings as dirty for delayed save
  PWM_MarkSettingsDirty();

  // Calculate and set pulse width
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2);
  uint32_t pulse = (period + 1) * current_duty_cycle / 100;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}

/**
 * @brief Get current PWM frequency
 * @return Current frequency in Hz
 */
uint32_t PWM_GetFrequency(void)
{
  return current_frequency;
}

/**
 * @brief Get current PWM duty cycle
 * @return Current duty cycle in percentage
 */
uint8_t PWM_GetDutyCycle(void)
{
  return current_duty_cycle;
}

/**
 * @brief Increase PWM frequency by one step
 */
void PWM_IncreaseFrequency(void)
{
  uint32_t new_freq = current_frequency + PWM_FREQ_STEP;
  if (new_freq <= PWM_MAX_FREQ)
  {
    PWM_SetFrequency(new_freq);
  }
}

/**
 * @brief Decrease PWM frequency by one step
 */
void PWM_DecreaseFrequency(void)
{
  uint32_t new_freq = current_frequency - PWM_FREQ_STEP;
  if (new_freq >= PWM_MIN_FREQ)
  {
    PWM_SetFrequency(new_freq);
  }
}

/**
 * @brief Increase PWM duty cycle by one step
 */
void PWM_IncreaseDutyCycle(void)
{
  uint8_t new_duty = current_duty_cycle + PWM_DUTY_STEP;
  if (new_duty <= 100)
  {
    PWM_SetDutyCycle(new_duty);
  }
}

/**
 * @brief Decrease PWM duty cycle by one step
 */
void PWM_DecreaseDutyCycle(void)
{
  int16_t new_duty = current_duty_cycle - PWM_DUTY_STEP;
  if (new_duty >= 0)
  {
    PWM_SetDutyCycle((uint8_t)new_duty);
  }
}
