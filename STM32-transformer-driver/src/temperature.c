#include "temperature.h"

// Microsecond delay function using DWT cycle counter
static void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = us * (SystemCoreClock / 1000000);
  while ((DWT->CYCCNT - start) < cycles)
    ;
}

// Set pin as output
static void DS18B20_GPIO_Output(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);
}

// Set pin as input
static void DS18B20_GPIO_Input(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);
}

// Write bit to DS18B20
static void DS18B20_WriteBit(uint8_t bit)
{
  DS18B20_GPIO_Output();

  if (bit)
  {
    // Write '1' bit
    HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
    delay_us(1);
    DS18B20_GPIO_Input();
    delay_us(70);
  }
  else
  {
    // Write '0' bit
    HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
    delay_us(70);
    DS18B20_GPIO_Input();
  }
}

// Read bit from DS18B20
static uint8_t DS18B20_ReadBit(void)
{
  uint8_t bit = 0;

  DS18B20_GPIO_Output();
  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
  delay_us(2);

  DS18B20_GPIO_Input();

  if (HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin))
  {
    bit = 1;
  }

  delay_us(60);

  return bit;
}

// Write byte to DS18B20
static void DS18B20_WriteByte(uint8_t byte)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    DS18B20_WriteBit(byte & 0x01);
    byte >>= 1;
  }
}

// Read byte from DS18B20
static uint8_t DS18B20_ReadByte(void)
{
  uint8_t byte = 0;

  for (uint8_t i = 0; i < 8; i++)
  {
    byte >>= 1;
    if (DS18B20_ReadBit())
    {
      byte |= 0x80;
    }
  }

  return byte;
}

// Initialize DS18B20 (1-Wire reset pulse)
static uint8_t DS18B20_Reset(void)
{
  uint8_t presence = 0;

  DS18B20_GPIO_Output();
  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
  delay_us(480); // Reset pulse

  DS18B20_GPIO_Input();
  delay_us(70); // Wait for presence pulse

  if (!HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin))
  {
    presence = 1; // Device present
  }

  delay_us(410); // Complete the reset sequence

  return presence;
}

// Initialize temperature module
void Temperature_Init(void)
{
  // Enable GPIO clock
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Enable DWT for microsecond delays
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Configure pin as input with pull-up (default state)
  DS18B20_GPIO_Input();
}

// Start temperature conversion
uint8_t Temperature_Start(void)
{
  if (!DS18B20_Reset())
  {
    return 0; // Device not present
  }

  DS18B20_WriteByte(DS18B20_CMD_SKIPROM);     // Skip ROM command
  DS18B20_WriteByte(DS18B20_CMD_CONVERTTEMP); // Start temperature conversion

  return 1; // Success
}

// Read temperature data from DS18B20 scratchpad
uint8_t Temperature_GetData(uint8_t *data)
{
  if (!DS18B20_Reset())
  {
    return 0; // Device not present
  }

  DS18B20_WriteByte(DS18B20_CMD_SKIPROM);     // Skip ROM command
  DS18B20_WriteByte(DS18B20_CMD_RSCRATCHPAD); // Read scratchpad

  // Read 2 bytes from scratchpad
  for (uint8_t i = 0; i < 2; i++)
  {
    data[i] = DS18B20_ReadByte();
  }

  return 1; // Success
}

// Read temperature and return as float (in Celsius)
float Temperature_Read(void)
{
  uint8_t data[2];
  volatile int16_t raw_temp;
  float temperature;

  // Start conversion
  if (!Temperature_Start())
  {
    return -1000.0f; // Error: device not present
  }

  // Read temperature data
  if (!Temperature_GetData(data))
  {
    return -1000.0f; // Error: device not present
  }

  // Calculate temperature
  raw_temp = (data[1] << 8) | data[0];
  temperature = (float)raw_temp / 16.0f;

  return temperature;
}
