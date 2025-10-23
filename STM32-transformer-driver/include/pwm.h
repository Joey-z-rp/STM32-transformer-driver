#ifndef PWM_H
#define PWM_H

#include "stm32f1xx_hal.h"

// PWM Configuration
#define PWM_MIN_FREQ 5000   // 5 kHz
#define PWM_MAX_FREQ 200000 // 200 kHz
#define PWM_FREQ_STEP 500   // Frequency step in Hz
#define PWM_DUTY_STEP 1     // Duty cycle step in %

// Function prototypes
void PWM_Init(void);
void PWM_SetFrequency(uint32_t frequency);
void PWM_SetDutyCycle(uint8_t duty_cycle);
uint32_t PWM_GetFrequency(void);
uint8_t PWM_GetDutyCycle(void);
void PWM_IncreaseFrequency(void);
void PWM_DecreaseFrequency(void);
void PWM_IncreaseDutyCycle(void);
void PWM_DecreaseDutyCycle(void);

// Settings persistence functions
void PWM_MarkSettingsDirty(void);
uint8_t PWM_AreSettingsDirty(void);
void PWM_ResetDirtyTimer(void);
uint32_t PWM_GetDirtyTimestamp(void);
void PWM_SaveSettings(void);

#endif // PWM_H
