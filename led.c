#include "led.h"

#include "91x_lib.h"


// =============================================================================
// Private data:

#define GREEN_LED_PIN (GPIO_Pin_6)
#define RED_LED_PIN (GPIO_Pin_7)


// =============================================================================
// Public functions:

void LEDInit(void)
{
  SCU_APBPeriphClockConfig(__GPIO5, ENABLE);  // Enable the GPIO5 Clock

  // The green LED is on pin GPIO5.6 and the red LED is on pin GPIO5.7
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GREEN_LED_PIN | RED_LED_PIN;
  GPIO_InitStructure.GPIO_Direction = GPIO_PinOutput;
  GPIO_InitStructure.GPIO_Type = GPIO_Type_PushPull;
  GPIO_InitStructure.GPIO_IPInputConnected = GPIO_IPInputConnected_Disable;
  GPIO_InitStructure.GPIO_Alternate = GPIO_OutputAlt1;
  GPIO_Init(GPIO5, &GPIO_InitStructure);

  GreenLEDOff();
  RedLEDOff();
}

#define LED_GRN_ON GPIO_WriteBit(GPIO5, GREEN_LED_PIN, Bit_SET)
#define LED_GRN_OFF GPIO_WriteBit(GPIO5, GREEN_LED_PIN, Bit_RESET)
#define LED_GRN_TOGGLE \
  if (GPIO_ReadBit(GPIO5, GREEN_LED_PIN)) \
    LED_GRN_OFF; \
  else LED_GRN_ON;

#define LED_RED_ON GPIO_WriteBit(GPIO5, RED_LED_PIN, Bit_SET)
#define LED_RED_OFF GPIO_WriteBit(GPIO5, RED_LED_PIN, Bit_RESET)
#define LED_RED_TOGGLE \
  if (GPIO_ReadBit(GPIO5, RED_LED_PIN)) \
    LED_RED_OFF; \
  else LED_RED_ON;

// -----------------------------------------------------------------------------
void GreenLEDOff(void)
{
  GPIO_WriteBit(GPIO5, GREEN_LED_PIN, Bit_SET);
}

// -----------------------------------------------------------------------------
void GreenLEDOn(void)
{
  GPIO_WriteBit(GPIO5, GREEN_LED_PIN, Bit_RESET);
}

// -----------------------------------------------------------------------------
void GreenLEDToggle(void)
{
  if (GPIO_ReadBit(GPIO5, GREEN_LED_PIN))
    GreenLEDOn();
  else
    GreenLEDOff();
}

// -----------------------------------------------------------------------------
void RedLEDOff(void)
{
  GPIO_WriteBit(GPIO5, RED_LED_PIN, Bit_RESET);
}

// -----------------------------------------------------------------------------
void RedLEDOn(void)
{
  GPIO_WriteBit(GPIO5, RED_LED_PIN, Bit_SET);
}

// -----------------------------------------------------------------------------
void RedLEDToggle(void)
{
  if (GPIO_ReadBit(GPIO5, RED_LED_PIN))
    RedLEDOff();
  else
    RedLEDOn();
}
