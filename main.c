#include "91x_lib.h"
#include "config.h"
#include "i2c.h"
#include "led.h"
#include "lsm303dl.h"
#include "spi_master.h"
#include "timing.h"
#include "uart.h"
#include "ublox.h"


// =============================================================================
// Private function declarations:

int main(void) __attribute__ ((noreturn));


// =============================================================================
// Private functions:

// Set the system clocks via the SCU (system control unit). The PLL (phase loop
// lock) will be the master clock, but it will be disabled while setting all of
// the other clocks before being re-enabled. For more information about the
// system clocks, see the STR91xFM reference manual section 2.4.
void SCUConfig(void)
{
  // Set master clock source to external oscillator (25MHz).
  SCU_MCLKSourceConfig(SCU_MCLK_OSC);
  // Configure the PLL to run at 48 MHz = (2 * N * f_OSC) / (M * 2 ^ P).
  // where feedback divider N = 192, pre-divider M = 25, & post-divider P = 3
  SCU_PLLFactorsConfig(192,25,3);
  // Enable the PLL.
  SCU_PLLCmd(ENABLE);
  // Set the RCLK (reference clock) divisor to 1.
  SCU_RCLKDivisorConfig(SCU_RCLK_Div1);
  // Set the HCLK (AHB (advanced high-performance bus) clock) divisor to 1.
  SCU_HCLKDivisorConfig(SCU_HCLK_Div1);
  // Set the PCLK (peripheral clock) divisor to 2.
  SCU_PCLKDivisorConfig(SCU_PCLK_Div2);
  // Set the PLL as the master clock source.
  SCU_MCLKSourceConfig(SCU_MCLK_PLL);
  // Set the BRCLK (baud rate clock) divisor to 2.
  SCU_BRCLKDivisorConfig(SCU_BRCLK_Div2);
}

//------------------------------------------------------------------------------
// Configure the interrupt vector.
void VICConfig(void)
{
  // Enable the AHB (advanced high-performance but) clock for VIC (vectored
  // interrupt controller).
  SCU_AHBPeriphClockConfig(__VIC,ENABLE);
  // Disable the reset state for the VIC.
  SCU_AHBPeriphReset(__VIC, DISABLE);
  // De-initialize the VIC module registers (to their default reset values).
  VIC_DeInit();
  // Initialize VICs default vector registers.
  VIC_InitDefaultVectors();
}

//------------------------------------------------------------------------------
int main(void)
{
  SCUConfig();
  VICConfig();
  TimingInit();
  LEDInit();
  UARTInit();
  I2CInit();
  SPIMasterInit();

  UBloxInit();
  LSM303DLInit();

  UARTPrintf("University of Tokyo NaviCtrl firmware V2");

  // Main loop.
  for (;;)
  {
    Wait(1000);
    ProcessIncomingUART();
    ProcessIncomingUBlox();
    LSM303DLReadMag();
    GreenLEDToggle();
    UARTPrintf("%i %i %i", Magnetometer()[0], Magnetometer()[1], Magnetometer()[2]);
  }
}
