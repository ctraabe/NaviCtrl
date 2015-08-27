#include <stdio.h>

#include "91x_lib.h"
#include "i2c.h"
#include "led.h"
#include "logging.h"
#include "lsm303dl.h"
#include "timing.h"
#include "uart.h"
#include "ublox.h"


// =============================================================================
// Private function declarations:

int main(void) __attribute__ ((noreturn));


// =============================================================================
// Private functions:

// Configure the interrupt vector.
void VICConfig(void)
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

  // Main loop.
  uint32_t led_timer = GetTimestamp();
  for (;;)
  {
    ProcessLogging();
    if (TimestampInPast(led_timer))
    {
      GreenLEDToggle();
      led_timer += 500;
    }
  }
}
