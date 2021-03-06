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

  GPIO_InitTypeDef gpio_init;

  // The green LED is on pin GPIO5.6 and the red LED is on pin GPIO5.7
  gpio_init.GPIO_Pin = GREEN_LED_PIN | RED_LED_PIN;
  gpio_init.GPIO_Direction = GPIO_PinOutput;
  gpio_init.GPIO_Type = GPIO_Type_PushPull;
  gpio_init.GPIO_IPInputConnected = GPIO_IPInputConnected_Disable;
  gpio_init.GPIO_Alternate = GPIO_OutputAlt1;
  GPIO_Init(GPIO5, &gpio_init);

  GreenLEDOff();
  RedLEDOff();
}

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
