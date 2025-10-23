#ifndef CURRENT_H
#define CURRENT_H

#include "stm32f1xx_hal.h"

/**
 * @file current.h
 * @brief ACS712 Current Sensor Driver
 *
 * This module provides functions to read current from an ACS712 Hall-effect
 * current sensor using the STM32's built-in ADC.
 *
 * Hardware Connection:
 * - Connect ACS712 OUT pin to PA6 (ADC1_IN6)
 * - Connect ACS712 VCC to 5V (or 3.3V with appropriate model)
 * - Connect ACS712 GND to GND
 * - Wire the load current through the ACS712 IP+ and IP- terminals
 *
 * Supported Models:
 * - ACS712-05A: ±5A range, 185 mV/A sensitivity
 * - ACS712-20A: ±20A range, 100 mV/A sensitivity
 * - ACS712-30A: ±30A range, 66 mV/A sensitivity
 *
 * Usage:
 * 1. Call Current_Init() to initialize the ADC module
 * 2. Call Current_Read() to get current in Amperes
 *    - Positive values indicate forward current
 *    - Negative values indicate reverse current
 */

// ACS712 Configuration
// Default pin: PA6 (ADC1_IN6)
#define ACS712_GPIO_Port GPIOA
#define ACS712_Pin GPIO_PIN_6
#define ACS712_ADC_CHANNEL ADC_CHANNEL_6

// ACS712 Model Selection (uncomment the one you're using)
// #define ACS712_MODEL_05A // ±5A, 185 mV/A
// #define ACS712_MODEL_20A    // ±20A, 100 mV/A
#define ACS712_MODEL_30A // ±30A, 66 mV/A

// Sensitivity values (mV/A)
#ifdef ACS712_MODEL_05A
#define ACS712_SENSITIVITY 0.185f // 185 mV/A
#define ACS712_MAX_CURRENT 5.0f
#elif defined(ACS712_MODEL_20A)
#define ACS712_SENSITIVITY 0.100f // 100 mV/A
#define ACS712_MAX_CURRENT 20.0f
#elif defined(ACS712_MODEL_30A)
#define ACS712_SENSITIVITY 0.066f // 66 mV/A
#define ACS712_MAX_CURRENT 30.0f
#else
#define ACS712_SENSITIVITY 0.185f // Default to 5A model
#define ACS712_MAX_CURRENT 5.0f
#endif

// ADC reference voltage (STM32F1 uses 3.3V)
#define ADC_VREF 3.3f

// Voltage divider ratio (if using voltage divider for 5V ACS712)
// For 10k + 20k divider: ratio = 20/(10+20) = 0.6667
// Set to 1.0f if connecting ACS712 output directly (3.3V powered ACS712)
#define VOLTAGE_DIVIDER_RATIO 0.6667f

// Zero current output voltage
// For 5V powered ACS712: 2.5V at sensor output
// After voltage divider: 2.5V * 0.6667 = 1.667V at ADC
#define ACS712_ZERO_CURRENT_VOLTAGE (2.5f * VOLTAGE_DIVIDER_RATIO)

// Number of samples for averaging (reduces noise)
#define CURRENT_ADC_SAMPLES 16

// Function prototypes
void Current_Init(void);
void Current_Calibrate(void);
float Current_Read(void);
uint16_t Current_ReadRaw(void);

#endif // CURRENT_H
