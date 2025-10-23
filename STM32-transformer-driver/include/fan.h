#ifndef FAN_H
#define FAN_H

#include "stm32f1xx_hal.h"

// Fan Configuration
#define FAN_TEMP_THRESHOLD 40.0f // Temperature threshold in Celsius to start fan
#define FAN_TEMP_MAX 60.0f       // Temperature for maximum fan speed
#define FAN_MIN_DUTY 20          // Minimum fan duty cycle when active (%)
#define FAN_MAX_DUTY 60          // Maximum fan duty cycle (%)
#define FAN_RAMP_STEP 5          // Duty cycle step for gradual ramping (%)
#define FAN_UPDATE_INTERVAL 500  // Update interval in milliseconds

// Function prototypes
void Fan_Init(void);
void Fan_Update(float temperature);
uint8_t Fan_GetDutyCycle(void);
uint8_t Fan_GetOutputPercent(void);
uint8_t Fan_IsActive(void);

#endif // FAN_H
