/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : 91x_it.h
* Author             : MCD Application Team
* Version            : V2.1
* Date               : 12/22/2008
* Description        : This file contains the headers of the interrupt
*                      handlers'routines
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifndef __91x_IT_H
#define __91x_IT_H

#include "91x_lib.h"


#define IENABLE \
  asm("MRS R0, SPSR\n\t");  /* Copy SPSR_irq to R0 */\
  asm("STMFD SP!, {R0}\n\t");  /* Save SPSR_irq */\
  asm("MSR CPSR_c, #0x1F\n\t");  /* Switch to SYS mode (interrupts enabled) */\
  asm("STMFD SP!, {LR}\n\t");  /* Save SYS_lr */

#define IDISABLE \
  asm("LDMFD SP!, {LR}\n\t");  /* Restore SYS_lr */\
  asm("MSR CPSR_c, #0x92\n\t");  /* Switch to IRQ mode (IRQ disabled) */\
  asm("LDMFD SP!, {R0}\n\t");  /* Restore SPSR_irq to R0 */\
  asm("MSR SPSR_cxsf, R0\n\t");  /* Copy R0 to SPSR_irq */

#define DAISY_VIC() \
  asm("MOV R11, #0xFC000000\n\t");  /* VIC1 base address */\
  asm("LDR R11, [R11, #0x30]\n\t");  /* Read VIC1-VAR (updates priority mask) */\
  asm("ADD R11 ,R11 , #0x14\n\t");  /* Skip the preamble (+0x14) */\
  asm("BX R11\n\t");  /* Branch to the highest priority interrupt from VIC1 */


void Undefined_Handler(void);
void SWI_Handler(void);
void Prefetch_Handler(void);
void Abort_Handler(void);
void FIQ_Handler(void);
void WDG_IRQHandler(void);
void SW_IRQHandler(void);
void ARMRX_IRQHandler(void);
void ARMTX_IRQHandler(void);
void TIM0_IRQHandler(void);
void TIM1_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void USBHP_IRQHandler(void);
void USBLP_IRQHandler(void);
void SCU_IRQHandler(void);
void ENET_IRQHandler(void);
void DMA_IRQHandler(void);
void CAN_IRQHandler(void);
void MC_IRQHandler(void);
void ADC_IRQHandler(void);
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
void UART2_IRQHandler(void);
void I2C0_IRQHandler(void);
void I2C1_IRQHandler(void);
void SSP0_IRQHandler(void);
void SSP1_IRQHandler(void);
void LVD_IRQHandler(void);
void RTC_IRQHandler(void);
void WIU_IRQHandler(void);
void EXTIT0_IRQHandler(void);
void EXTIT1_IRQHandler(void);
void EXTIT2_IRQHandler(void);
void EXTIT3_IRQHandler(void);
void USBWU_IRQHandler(void);
void PFQBC_IRQHandler(void);
void DefaultVector_Handler(void);

#endif /* __91x_IT_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
