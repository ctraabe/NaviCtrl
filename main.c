#include "91x_lib.h"
#include "i2c.h"
#include "irq_priority.h"
#include "led.h"
#include "logging.h"
#include "lsm303dl.h"
#include "timing.h"
#include "uart.h"
#include "ublox.h"


// =============================================================================
// Private functions:

// Configure the interrupt vector.
static void VICConfig(void)
{
  // Enable the AHB (advanced high-performance but) clock for VIC (vectored
  // interrupt controller).
  SCU_AHBPeriphClockConfig(__VIC,ENABLE);
  // Reset the VIC registers (to their default reset values).
  VIC_DeInit();
  // Initialize VICs default vector registers.
  VIC_InitDefaultVectors();
}

//------------------------------------------------------------------------------
static void ExternalButtonInit(void)
{
  SCU_APBPeriphClockConfig(__GPIO3 ,ENABLE);

  GPIO_InitTypeDef gpio_init;

  // Configure P3.1 -> external button as an output pin.
  gpio_init.GPIO_Direction = GPIO_PinInput;
  gpio_init.GPIO_Pin = GPIO_Pin_1;
  gpio_init.GPIO_Type = GPIO_Type_PushPull;
  gpio_init.GPIO_IPInputConnected = GPIO_IPInputConnected_Disable;
  gpio_init.GPIO_Alternate = GPIO_InputAlt1;
  GPIO_Init (GPIO3, &gpio_init);
}

//------------------------------------------------------------------------------
int main(void) __attribute__ ((noreturn));
int main(void)
{
  VICConfig();
  TimingInit();
  LEDInit();
  UARTInit();
  I2CInit();

  UARTPrintf("University of Tokyo NaviCtrl firmware V2");

  UBloxInit();
  LSM303DLInit();
  LoggingInit();

  ExternalButtonInit();

  // Enable the "new data" Interrupt.
  VIC_Config(EXTIT1_ITLine, VIC_IRQ, IRQ_PRIORITY_NEW_DATA);
  VIC_ITCmd(EXTIT1_ITLine, ENABLE);

  // Enable the 50Hz Interrupt.
  VIC_Config(EXTIT2_ITLine, VIC_IRQ, IRQ_PRIORITY_50HZ);
  VIC_ITCmd(EXTIT2_ITLine, ENABLE);

  // Main loop.
  // uint32_t led_timer = GetTimestamp();
  for (;;)
  {
    ProcessLoggingSlow();

    // if (TimestampInPast(led_timer))
    // {
    //   GreenLEDToggle();
    //   led_timer += 500;
    // }
  }
}

//------------------------------------------------------------------------------
// New data received
void EXTIT1_IRQHandler(void)
{
  DAISY_VIC();
  IENABLE;
  VIC_SWITCmd(EXTIT1_ITLine, DISABLE);

  ProcessLoggingFast();

  IDISABLE;
  VIC1->VAR = 0xFF;
}

//------------------------------------------------------------------------------
// 50 Hz interrupt
void EXTIT2_IRQHandler(void)
{
  DAISY_VIC();
  IENABLE;
  VIC_SWITCmd(EXTIT2_ITLine, DISABLE);

  uint16_t button = GPIO_ReadBit(GPIO3, GPIO_Pin_1);
  static uint16_t button_pv = 0;
  if (button && (button_pv == 0x7FFF))
  {
    if (!LoggingActive()) OpenLogFile("test.csv");
    else CloseLogFile();
  }
  else
  {
    LSM303DLReadMag();
    ProcessIncomingUART();
  }
  button_pv = (button_pv << 1) | button;

  IDISABLE;
  VIC1->VAR = 0xFF;
}
