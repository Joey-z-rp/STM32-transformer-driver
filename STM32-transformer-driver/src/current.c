#include "current.h"

// ADC handle
static ADC_HandleTypeDef hadc1;

// Calibration offset (current reading when no current is flowing)
static float calibration_offset = 0.0f;

/**
 * @brief Initialize ADC for current sensing
 */
void Current_Init(void)
{
  // Enable ADC1 and GPIO clock
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Configure GPIO pin as analog input
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = ACS712_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(ACS712_GPIO_Port, &GPIO_InitStruct);

  // Configure ADC
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    // Initialization error
    return;
  }

  // Calibrate ADC (important for accuracy on STM32F1)
  HAL_ADCEx_Calibration_Start(&hadc1);
}

/**
 * @brief Read raw ADC value (0-4095 for 12-bit ADC)
 * @return ADC value (0-4095)
 */
uint16_t Current_ReadRaw(void)
{
  uint32_t adc_sum = 0;
  ADC_ChannelConfTypeDef sConfig = {0};

  // Configure ADC channel
  sConfig.Channel = ACS712_ADC_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5; // Slower sampling for better accuracy

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    return 0;
  }

  // Take multiple samples and average them
  for (uint8_t i = 0; i < CURRENT_ADC_SAMPLES; i++)
  {
    // Start ADC conversion
    HAL_ADC_Start(&hadc1);

    // Wait for conversion to complete
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
      adc_sum += HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);
  }

  // Return average
  return (uint16_t)(adc_sum / CURRENT_ADC_SAMPLES);
}

/**
 * @brief Read current from ACS712 sensor
 * @return Current in Amperes (positive = forward, negative = reverse)
 */
float Current_Read(void)
{
  // Read ADC value
  uint16_t adc_value = Current_ReadRaw();

  // Convert ADC value to voltage at ADC pin
  // For 12-bit ADC: voltage = (adc_value / 4095) * VREF
  float voltage_at_adc = ((float)adc_value / 4095.0f) * ADC_VREF;

  // Calculate current based on ACS712 characteristics
  // Current = (Vout - Vzero) / Sensitivity
  // Note: Both voltage and sensitivity are scaled by voltage divider ratio
  // The ratio cancels out: (V*ratio - Vzero*ratio) / (Sens*ratio) = (V - Vzero) / Sens
  float current = (voltage_at_adc - ACS712_ZERO_CURRENT_VOLTAGE) / (ACS712_SENSITIVITY * VOLTAGE_DIVIDER_RATIO);

  // Apply calibration offset
  current -= calibration_offset;

  return current;
}

/**
 * @brief Calibrate the current sensor at zero current
 * @note Call this function when you know there is no current flowing (e.g., PWM duty = 0)
 */
void Current_Calibrate(void)
{
  // Take multiple readings and average them for better accuracy
  float sum = 0.0f;
  const uint8_t num_samples = 10; // More samples for calibration

  for (uint8_t i = 0; i < num_samples; i++)
  {
    // Read ADC value
    uint16_t adc_value = Current_ReadRaw();

    // Convert to voltage
    float voltage_at_adc = ((float)adc_value / 4095.0f) * ADC_VREF;

    // Calculate uncalibrated current
    float current = (voltage_at_adc - ACS712_ZERO_CURRENT_VOLTAGE) / (ACS712_SENSITIVITY * VOLTAGE_DIVIDER_RATIO);

    sum += current;

    // Small delay between samples
    HAL_Delay(100);
  }

  // Store the average as the calibration offset
  calibration_offset = sum / num_samples;
}
