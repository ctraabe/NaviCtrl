#include "uart.h"

#include <stdarg.h>
#include <stdio.h>

#include "91x_lib.h"
#include "config.h"


// =============================================================================
// Private data:

#define UART_BAUD (57600)


// =============================================================================
// Public functions:

void UARTInit(void)
{
  UART_InitTypeDef UART_InitStructure;

  UART_InitStructure.UART_WordLength = UART_WordLength_8D;
  UART_InitStructure.UART_StopBits = UART_StopBits_1;
  UART_InitStructure.UART_Parity = UART_Parity_No ;
  UART_InitStructure.UART_BaudRate = UART_BAUD;
  UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
  UART_InitStructure.UART_Mode = UART_Mode_Tx_Rx;
  UART_InitStructure.UART_FIFO = UART_FIFO_Disable;
  UART_InitStructure.UART_TxFIFOLevel = UART_FIFOLevel_1_2;
  UART_InitStructure.UART_RxFIFOLevel = UART_FIFOLevel_1_2;

  UART_DeInit(UART1);
  UART_Init(UART1, &UART_InitStructure);

  // Enable UART Rx interrupt.
  UART_ITConfig(UART1, UART_IT_Receive , ENABLE);
  VIC_Config(UART1_ITLine, VIC_IRQ, PRIORITY_UART1);

  UART_Cmd(UART1, ENABLE);

  // Enable UART1 interrupt for data reception.
  VIC_ITCmd(UART1_ITLine, ENABLE);
}

// -----------------------------------------------------------------------------
// This function mimics printf, but puts the result on the UART stream. It also
// adds the end-of-line characters and checks that the character buffer is not
// exceeded. Note that this function is slow and blocking.
void UARTPrintf_P(const char *format, ...)
{
  uint8_t ascii[103];  // 100 chars + 2 newline chars + null terminator

  va_list arglist;
  va_start(arglist, format);
  int length = vsnprintf((char *)ascii, 101, (char *)format, arglist);
  va_end(arglist);

  if (length < 101)
    sprintf((char *)&ascii[length], "\n\r");
  else
    sprintf((char *)&ascii[80], "... MESSAGE TOO LONG\n\r");

  uint8_t *pointer = &ascii[0];
  while (*pointer)
  {
    UART_SendData(UART1, *pointer++);
    while(UART_GetFlagStatus(UART1, UART_FLAG_TxFIFOFull) != RESET);
  }
}


// =============================================================================
// Private functions:

void UART1_IRQHandler(void)
{
  // Clear receive interrupt flag.
  UART_ClearITPendingBit(UART1, UART_IT_Receive);

  // Send back the received character.
  while (UART_GetFlagStatus(UART1, UART_FLAG_TxFIFOFull) != RESET);
  UART_SendData(UART1, UART_ReceiveData(UART1));
}
