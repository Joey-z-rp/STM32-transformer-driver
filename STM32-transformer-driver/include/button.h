#ifndef BUTTON_H
#define BUTTON_H

#include "stm32f1xx_hal.h"

// Button pin definitions
#define BTN_FREQ_UP_PIN GPIO_PIN_1
#define BTN_FREQ_DOWN_PIN GPIO_PIN_2
#define BTN_DUTY_UP_PIN GPIO_PIN_3
#define BTN_DUTY_DOWN_PIN GPIO_PIN_4
#define BTN_GPIO_PORT GPIOA

// Debounce time in milliseconds
#define DEBOUNCE_TIME 50

// Auto-repeat timing configuration (all in milliseconds)
#define INITIAL_HOLD_DELAY 500   // Delay before auto-repeat starts
#define REPEAT_INTERVAL_SLOW 200 // Initial repeat interval (slow)
#define REPEAT_INTERVAL_FAST 30  // Final repeat interval (fast)
#define ACCELERATION_TIME 2000   // Time to transition from slow to fast

// Button state enumeration
typedef enum
{
  BTN_RELEASED = 0,
  BTN_PRESSED = 1
} ButtonState_t;

// Function prototypes
void Button_Init(void);
void Button_Process(void);

#endif // BUTTON_H
