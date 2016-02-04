/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : 91x_it.c
* Author             : MCD Application Team
* Version            : V2.1
* Date               : 12/22/2008
* Description        : Main Interrupt Service Routines.
*                      This file can be used to describe all the exceptions
*                      subroutines that may occur within user application. When
*                      an interrupt happens, the software will branch
*                      automatically to the corresponding routine. The following
*                      routines are all empty, user can write code  for
*                      exceptions handlers and peripherals IRQ interrupts.
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#include "91x_it.h"

#include "flight_ctrl_comms.h"
#include "i2c.h"
#include "led.h"
#include "logging.h"
#include "main.h"
#include "sd_card.h"
#include "spi_slave.h"
#include "uart.h"
#include "ublox.h"


// Exceptions:
void Undefined_Handler(void) { RedLEDOn(); for (;;) continue; }
void SWI_Handler(void) { RedLEDOn(); for (;;) continue; }
void Prefetch_Handler(void) { RedLEDOn(); for (;;) continue; }
void Abort_Handler(void) { RedLEDOn(); for (;;) continue; }

// Fast interrupt:
void FIQ_Handler(void) { for (;;) continue; }

// VIC 0:
void WDG_IRQHandler(void) { VIC0->VAR = 0xFF; }
void SW_IRQHandler(void) { VIC0->VAR = 0xFF; }
void ARMRX_IRQHandler(void) { VIC0->VAR = 0xFF; }
void ARMTX_IRQHandler(void) { VIC0->VAR = 0xFF; }
void TIM0_IRQHandler(void) { VIC0->VAR = 0xFF; }
// void TIM1_IRQHandler(void) { VIC0->VAR = 0xFF; }  // Moved to timing.c
void TIM2_IRQHandler(void) { VIC0->VAR = 0xFF; }
void TIM3_IRQHandler(void) { VIC0->VAR = 0xFF; }
void USBHP_IRQHandler(void) { VIC0->VAR = 0xFF; }
void USBLP_IRQHandler(void) { VIC0->VAR = 0xFF; }
void SCU_IRQHandler(void) { VIC0->VAR = 0xFF; }
void ENET_IRQHandler(void) { VIC0->VAR = 0xFF; }
void DMA_IRQHandler(void) { VIC0->VAR = 0xFF; }
void CAN_IRQHandler(void) { VIC0->VAR = 0xFF; }
void MC_IRQHandler(void) { VIC0->VAR = 0xFF; }
void ADC_IRQHandler(void) { VIC0->VAR = 0xFF; }

// VIC 1:
void UART0_IRQHandler(void)
{
  DAISY_VIC();
  IENABLE;

  UBloxUARTHandler();

  IDISABLE;
  VIC1->VAR = 0xFF;
}
void UART1_IRQHandler(void)
{
  DAISY_VIC();
  IENABLE;

  UARTHandler();

  IDISABLE;
  VIC1->VAR = 0xFF;
}
void UART2_IRQHandler(void) { VIC1->VAR = 0xFF; }
void I2C0_IRQHandler(void) { VIC1->VAR = 0xFF; }
void I2C1_IRQHandler(void)
{
  DAISY_VIC();
  IENABLE;

  I2CHandler();

  IDISABLE;
  VIC1->VAR = 0xFF;
}
void SSP0_IRQHandler(void)
{
  DAISY_VIC();
  IENABLE;

  SPISlaveHandler();

  IDISABLE;
  VIC1->VAR = 0xFF;
}
void SSP1_IRQHandler(void)
{
  DAISY_VIC();
  IENABLE;

  SDSPIHandler();

  IDISABLE;
  VIC1->VAR = 0xFF;
}
void LVD_IRQHandler(void) { VIC1->VAR = 0xFF; }
void RTC_IRQHandler(void) { VIC1->VAR = 0xFF; }
void WIU_IRQHandler(void) { VIC1->VAR = 0xFF; }
void EXTIT0_IRQHandler(void)
{
  DAISY_VIC();
  IENABLE;

  NewDataInterruptHandler();

  IDISABLE;
  VIC1->VAR = 0xFF;
}
void EXTIT1_IRQHandler(void)
{
  DAISY_VIC();
  IENABLE;

  SDCardPresentHandler();

  IDISABLE;
  VIC1->VAR = 0xFF;
}
void EXTIT2_IRQHandler(void)
{
  DAISY_VIC();
  IENABLE;

  FlightCtrlInterruptHandler();

  IDISABLE;
  VIC1->VAR = 0xFF;
}
void EXTIT3_IRQHandler(void)
{
  DAISY_VIC();
  IENABLE;

  FiftyHzInterruptHandler();

  IDISABLE;
  VIC1->VAR = 0xFF;
}
void USBWU_IRQHandler(void) { VIC1->VAR = 0xFF; }
void PFQBC_IRQHandler(void) { VIC1->VAR = 0xFF; }
void DefaultVector_Handler(void)
{
  VIC0->VAR = 0xFF;
  VIC1->VAR = 0xFF;
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
