#include "button.h"
#include "pwm.h"

// Button state tracking structure
typedef struct
{
  uint32_t debounce_time;    // Last time for debouncing
  uint32_t press_time;       // Time when button was first pressed
  uint32_t last_action_time; // Time when last action was executed
  ButtonState_t state;       // Current button state
  uint8_t is_holding;        // Flag indicating button is being held
} ButtonContext_t;

// Button contexts for each button
static ButtonContext_t btn_freq_up = {0};
static ButtonContext_t btn_freq_down = {0};
static ButtonContext_t btn_duty_up = {0};
static ButtonContext_t btn_duty_down = {0};

/**
 * @brief Initialize buttons as inputs with pull-up resistors
 */
void Button_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable GPIOA clock (if not already enabled)
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Configure button pins as inputs with pull-up resistors
  // Buttons are active LOW (pressed = LOW, released = HIGH)
  GPIO_InitStruct.Pin = BTN_FREQ_UP_PIN | BTN_FREQ_DOWN_PIN |
                        BTN_DUTY_UP_PIN | BTN_DUTY_DOWN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BTN_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief Read current button state (with debouncing)
 * @param pin Button GPIO pin
 * @param context Button context structure
 * @return Current debounced button state
 */
static ButtonState_t Button_ReadDebounced(uint16_t pin, ButtonContext_t *context)
{
  uint32_t current_time = HAL_GetTick();
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(BTN_GPIO_PORT, pin);

  // Button is active LOW
  ButtonState_t current_state = (pin_state == GPIO_PIN_RESET) ? BTN_PRESSED : BTN_RELEASED;

  // Check if button state changed
  if (current_state != context->state)
  {
    // Check if enough time has passed since we last accepted a state change
    if ((current_time - context->debounce_time) >= DEBOUNCE_TIME)
    {
      // Accept the state change
      context->state = current_state;
      context->debounce_time = current_time;

      // If button was just pressed, record the press time
      if (current_state == BTN_PRESSED)
      {
        context->press_time = current_time;
        context->last_action_time = current_time;
        context->is_holding = 0;
      }
      else
      {
        // Button released, reset holding flag
        context->is_holding = 0;
      }
    }
  }
  else
  {
    // State hasn't changed, so reset the debounce timer
    context->debounce_time = current_time;
  }

  return context->state;
}

/**
 * @brief Calculate current repeat interval based on hold duration
 * @param hold_duration Time button has been held (in milliseconds)
 * @return Current repeat interval in milliseconds
 */
static uint32_t Button_GetRepeatInterval(uint32_t hold_duration)
{
  // If not past initial delay, return a large value to prevent repeat
  if (hold_duration < INITIAL_HOLD_DELAY)
  {
    return UINT32_MAX;
  }

  // Calculate time since auto-repeat started
  uint32_t repeat_duration = hold_duration - INITIAL_HOLD_DELAY;

  // If still in acceleration phase, interpolate between slow and fast
  if (repeat_duration < ACCELERATION_TIME)
  {
    // Linear interpolation from slow to fast
    uint32_t interval_range = REPEAT_INTERVAL_SLOW - REPEAT_INTERVAL_FAST;
    uint32_t interval_reduction = (interval_range * repeat_duration) / ACCELERATION_TIME;
    return REPEAT_INTERVAL_SLOW - interval_reduction;
  }

  // Acceleration complete, use fast interval
  return REPEAT_INTERVAL_FAST;
}

/**
 * @brief Process a single button with auto-repeat and acceleration
 * @param pin Button GPIO pin
 * @param context Button context structure
 * @param action Function to call when button action should occur
 */
static void Button_ProcessSingle(uint16_t pin, ButtonContext_t *context, void (*action)(void))
{
  ButtonState_t state = Button_ReadDebounced(pin, context);
  uint32_t current_time = HAL_GetTick();

  if (state == BTN_PRESSED)
  {
    uint32_t hold_duration = current_time - context->press_time;
    uint32_t time_since_action = current_time - context->last_action_time;

    // Execute action on initial press
    if (!context->is_holding)
    {
      action();
      context->is_holding = 1;
      context->last_action_time = current_time;
    }
    // Execute action if enough time has passed based on current repeat interval
    else
    {
      uint32_t repeat_interval = Button_GetRepeatInterval(hold_duration);

      if (time_since_action >= repeat_interval)
      {
        action();
        context->last_action_time = current_time;
      }
    }
  }
}

/**
 * @brief Process button inputs and update PWM settings
 * Call this function periodically in the main loop
 */
void Button_Process(void)
{
  // Process each button with auto-repeat
  Button_ProcessSingle(BTN_FREQ_UP_PIN, &btn_freq_up, PWM_IncreaseFrequency);
  Button_ProcessSingle(BTN_FREQ_DOWN_PIN, &btn_freq_down, PWM_DecreaseFrequency);
  Button_ProcessSingle(BTN_DUTY_UP_PIN, &btn_duty_up, PWM_IncreaseDutyCycle);
  Button_ProcessSingle(BTN_DUTY_DOWN_PIN, &btn_duty_down, PWM_DecreaseDutyCycle);
}
