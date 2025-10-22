#include "screen.h"
#include "i2c.h"
#include <stdio.h>
#include <string.h>

// LCD Commands
#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_CURSOR_SHIFT 0x10
#define LCD_FUNCTION_SET 0x20
#define LCD_SET_CGRAM_ADDR 0x40
#define LCD_SET_DDRAM_ADDR 0x80

// Flags for display entry mode
#define LCD_ENTRY_RIGHT 0x00
#define LCD_ENTRY_LEFT 0x02
#define LCD_ENTRY_SHIFT_INCREMENT 0x01
#define LCD_ENTRY_SHIFT_DECREMENT 0x00

// Flags for display on/off control
#define LCD_DISPLAY_ON 0x04
#define LCD_DISPLAY_OFF 0x00
#define LCD_CURSOR_ON 0x02
#define LCD_CURSOR_OFF 0x00
#define LCD_BLINK_ON 0x01
#define LCD_BLINK_OFF 0x00

// Flags for function set
#define LCD_8BIT_MODE 0x10
#define LCD_4BIT_MODE 0x00
#define LCD_2_LINE 0x08
#define LCD_1_LINE 0x00
#define LCD_5x10_DOTS 0x04
#define LCD_5x8_DOTS 0x00

// PCF8574 pin mapping
#define LCD_RS 0x01 // P0
#define LCD_RW 0x02 // P1
#define LCD_EN 0x04 // P2
#define LCD_BL 0x08 // P3 (Backlight)

// Global backlight state
static uint8_t backlight_state = LCD_BL;

/**
 * @brief Send nibble (4 bits) to LCD via PCF8574
 * @param nibble 4-bit data in upper nibble (D4-D7)
 * @param mode RS pin state (0=command, 1=data)
 */
static void LCD_SendNibble(uint8_t nibble, uint8_t mode)
{
  uint8_t data = nibble | mode | backlight_state;

  // Send with EN high
  uint8_t buf_high = data | LCD_EN;
  I2C_Write(LCD_I2C_ADDR, &buf_high, 1);
  HAL_Delay(1);

  // Send with EN low (latch data)
  uint8_t buf_low = data & ~LCD_EN;
  I2C_Write(LCD_I2C_ADDR, &buf_low, 1);
  HAL_Delay(1);
}

/**
 * @brief Send byte to LCD in 4-bit mode
 * @param data Data byte to send
 * @param mode RS pin state (0=command, 1=data)
 */
static void LCD_SendByte(uint8_t data, uint8_t mode)
{
  // Send high nibble
  LCD_SendNibble(data & 0xF0, mode);

  // Send low nibble
  LCD_SendNibble((data << 4) & 0xF0, mode);
}

/**
 * @brief Send command to LCD
 * @param cmd Command byte
 */
static void LCD_SendCommand(uint8_t cmd)
{
  LCD_SendByte(cmd, 0); // RS = 0 for command
  if (cmd == LCD_CLEAR_DISPLAY || cmd == LCD_RETURN_HOME)
  {
    HAL_Delay(2); // These commands need more time
  }
}

/**
 * @brief Send data (character) to LCD
 * @param data Character to display
 */
static void LCD_SendData(uint8_t data)
{
  LCD_SendByte(data, LCD_RS); // RS = 1 for data
}

/**
 * @brief Initialize LCD1602 in 4-bit mode
 */
void Screen_Init(void)
{
  // Initialize I2C first
  I2C_Init();

  // Wait for LCD to power up
  HAL_Delay(50);

  // Initialize LCD in 4-bit mode
  // Send function set commands (initialization sequence for 4-bit mode)
  LCD_SendNibble(0x30, 0);
  HAL_Delay(5);
  LCD_SendNibble(0x30, 0);
  HAL_Delay(1);
  LCD_SendNibble(0x30, 0);
  HAL_Delay(1);

  // Switch to 4-bit mode
  LCD_SendNibble(0x20, 0);
  HAL_Delay(1);

  // Function set: 4-bit mode, 2 lines, 5x8 dots
  LCD_SendCommand(LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2_LINE | LCD_5x8_DOTS);

  // Display control: display on, cursor off, blink off
  LCD_SendCommand(LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);

  // Clear display
  LCD_SendCommand(LCD_CLEAR_DISPLAY);

  // Entry mode: increment cursor, no shift
  LCD_SendCommand(LCD_ENTRY_MODE_SET | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT);

  HAL_Delay(2);
}

/**
 * @brief Clear LCD display
 */
void Screen_Clear(void)
{
  LCD_SendCommand(LCD_CLEAR_DISPLAY);
  HAL_Delay(2);
}

/**
 * @brief Set cursor position
 * @param row Row number (0 or 1)
 * @param col Column number (0 to 15)
 */
void Screen_SetCursor(uint8_t row, uint8_t col)
{
  // DDRAM addresses: Row 0 = 0x00-0x0F, Row 1 = 0x40-0x4F
  uint8_t row_offsets[] = {0x00, 0x40};

  if (row >= LCD_ROWS)
    row = LCD_ROWS - 1;
  if (col >= LCD_COLS)
    col = LCD_COLS - 1;

  LCD_SendCommand(LCD_SET_DDRAM_ADDR | (col + row_offsets[row]));
}

/**
 * @brief Print string to LCD at current cursor position
 * @param str Null-terminated string to print
 */
void Screen_Print(const char *str)
{
  while (*str)
  {
    LCD_SendData(*str++);
  }
}

/**
 * @brief Print unsigned integer to LCD
 * @param number Number to print
 */
void Screen_PrintNumber(uint32_t number)
{
  char buffer[12];
  sprintf(buffer, "%lu", number);
  Screen_Print(buffer);
}

/**
 * @brief Control LCD backlight
 * @param state 1=ON, 0=OFF
 */
void Screen_Backlight(uint8_t state)
{
  backlight_state = state ? LCD_BL : 0;

  // Send current state to update backlight
  uint8_t data = backlight_state;
  I2C_Write(LCD_I2C_ADDR, &data, 1);
}

/**
 * @brief Update only the frequency value on LCD (no screen clear)
 * @param frequency PWM frequency in Hz
 */
void Screen_UpdateFrequency(uint32_t frequency)
{
  char buffer[20]; // Larger buffer to avoid overflow warning

  // Position cursor after "Freq:" label (column 5)
  Screen_SetCursor(0, 5);

  // Format frequency with appropriate unit
  if (frequency >= 1000)
  {
    // Display in kHz
    uint32_t khz = frequency / 1000;
    uint32_t hz_remainder = (frequency % 1000) / 100; // One decimal place
    snprintf(buffer, sizeof(buffer), "%lu.%lu kHz    ", khz, hz_remainder);
  }
  else
  {
    // Display in Hz
    snprintf(buffer, sizeof(buffer), "%lu Hz     ", frequency);
  }
  Screen_Print(buffer);
}

/**
 * @brief Update only the duty cycle value on LCD (no screen clear)
 * @param duty_cycle PWM duty cycle in percentage (0-100)
 */
void Screen_UpdateDutyCycle(uint8_t duty_cycle)
{
  char buffer[20]; // Larger buffer to avoid overflow warning

  // Position cursor after "Duty:" label (column 5)
  Screen_SetCursor(1, 5);
  snprintf(buffer, sizeof(buffer), "%u%%     ", duty_cycle);
  Screen_Print(buffer);
}

/**
 * @brief Update LCD with current PWM frequency and duty cycle (full screen refresh)
 * @param frequency PWM frequency in Hz
 * @param duty_cycle PWM duty cycle in percentage (0-100)
 */
void Screen_UpdatePWM(uint32_t frequency, uint8_t duty_cycle)
{
  char buffer[20]; // Larger buffer to avoid overflow warning

  // Clear display
  Screen_Clear();

  // Line 1: Display frequency
  Screen_SetCursor(0, 0);
  Screen_Print("Freq:");

  // Format frequency with appropriate unit
  if (frequency >= 1000)
  {
    // Display in kHz
    uint32_t khz = frequency / 1000;
    uint32_t hz_remainder = (frequency % 1000) / 100; // One decimal place
    snprintf(buffer, sizeof(buffer), "%lu.%lu kHz", khz, hz_remainder);
  }
  else
  {
    // Display in Hz
    snprintf(buffer, sizeof(buffer), "%lu Hz", frequency);
  }
  Screen_Print(buffer);

  // Line 2: Display duty cycle
  Screen_SetCursor(1, 0);
  Screen_Print("Duty:");
  snprintf(buffer, sizeof(buffer), "%u%%", duty_cycle);
  Screen_Print(buffer);
}

/**
 * @brief Update only the temperature value on LCD (no screen clear)
 * @param temperature Temperature in Celsius
 */
void Screen_UpdateTemperature(float temperature)
{
  char buffer[20];

  // Position cursor after "Temp:" label (column 6 on line 2)
  Screen_SetCursor(1, 5);
  snprintf(buffer, sizeof(buffer), "%.1fC    ", temperature);
  Screen_Print(buffer);
}

/**
 * @brief Display PWM frequency and temperature on LCD
 * @param frequency PWM frequency in Hz
 * @param duty_cycle PWM duty cycle in percentage (0-100)
 * @param temperature Temperature in Celsius
 */
void Screen_DisplayPWMandTemp(uint32_t frequency, uint8_t duty_cycle, float temperature)
{
  char buffer[20];

  // Clear display
  Screen_Clear();

  // Line 1: Display frequency and duty cycle (compact format)
  Screen_SetCursor(0, 0);

  // Format frequency
  if (frequency >= 1000)
  {
    uint32_t khz = frequency / 1000;
    snprintf(buffer, sizeof(buffer), "F:%lukHz D:%u%%", khz, duty_cycle);
  }
  else
  {
    snprintf(buffer, sizeof(buffer), "F:%luHz D:%u%%", frequency, duty_cycle);
  }
  Screen_Print(buffer);

  // Line 2: Display temperature
  Screen_SetCursor(1, 0);
  Screen_Print("Temp:");
  snprintf(buffer, sizeof(buffer), "%.1fC", temperature);
  Screen_Print(buffer);
}
