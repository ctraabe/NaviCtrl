#include "uart.h"

#include <stdarg.h>
#include <stdio.h>

#include "91x_lib.h"
#include "config.h"
#include "mk_serial_protocol.h"
#include "mk_serial_tx.h"


// =============================================================================
// Private data:

#define UART_BAUD (57600)

static volatile uint8_t rx_buffer_[UART_RX_BUFFER_LENGTH], *tx_ptr_ = 0;
static volatile size_t rx_buffer_head_ = 0, tx_bytes_remaining_ = 0;
static uint8_t data_buffer_[UART_DATA_BUFFER_LENGTH];
static uint8_t tx_buffer_[UART_TX_BUFFER_LENGTH];
static uint8_t tx_overflow_counter_ = 0;


// =============================================================================
// Private function declarations:

static void ReceiveUARTData(void);


// =============================================================================
// Public functions:

void UARTInit(void)
{
  SCU_APBPeriphClockConfig(__GPIO3, ENABLE);  // Enable the GPIO3 Clock
  SCU_APBPeriphClockConfig(__UART1, ENABLE);  // Enable the UART1 Clock

  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure pin GPIO3.2 to be UART1 Rx
  GPIO_InitStructure.GPIO_Direction = GPIO_PinInput;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Type = GPIO_Type_PushPull;
  GPIO_InitStructure.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
  GPIO_InitStructure.GPIO_Alternate = GPIO_InputAlt1;  // UART1 Rx
  GPIO_Init(GPIO3, &GPIO_InitStructure);

  // Configure pin GPIO3.3 to be UART1 Tx
  GPIO_InitStructure.GPIO_Direction = GPIO_PinOutput;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Type = GPIO_Type_PushPull;
  GPIO_InitStructure.GPIO_Alternate = GPIO_OutputAlt3;  // UART1 Tx
  GPIO_Init(GPIO3, &GPIO_InitStructure);

  UART_InitTypeDef UART_InitStructure;

  UART_InitStructure.UART_WordLength = UART_WordLength_8D;
  UART_InitStructure.UART_StopBits = UART_StopBits_1;
  UART_InitStructure.UART_Parity = UART_Parity_No ;
  UART_InitStructure.UART_BaudRate = UART_BAUD;
  UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
  UART_InitStructure.UART_Mode = UART_Mode_Tx_Rx;
  UART_InitStructure.UART_FIFO = UART_FIFO_Enable;
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
// This function processes bytes that have been read into the Rx ring buffer
// (rx_buffer_) by the Rx interrupt handler. Each byte is passed to the
// appropriate Rx handler, which may place it into the temporary data buffer
// (data_buffer_).
void ProcessIncomingUART(void)
{
  static size_t rx_buffer_tail = 0;
  static enum UARTRxMode mode = UART_RX_MODE_IDLE;

  ReceiveUARTData();

  while (rx_buffer_tail != rx_buffer_head_)
  {
    // Move the ring buffer tail forward.
    rx_buffer_tail = (rx_buffer_tail + 1) % UART_RX_BUFFER_LENGTH;

    // Add other Rx protocols here.
    if (mode != UART_RX_MODE_IDLE)
      mode = MKSerialRx(rx_buffer_[rx_buffer_tail], data_buffer_);
    else if (rx_buffer_[rx_buffer_tail] == '#')  // MK protocol start character
      mode = UART_RX_MODE_MK_ONGOING;
  }
}

// -----------------------------------------------------------------------------
// This function returns the address of the shared Tx buffer (tx_buffer_) if it
// is available of zero if not.
uint8_t * RequestUARTTxBuffer(void)
{
  if (tx_bytes_remaining_)
  {
    tx_overflow_counter_++;
    return 0;
  }
  return tx_buffer_;
}

// -----------------------------------------------------------------------------
// This function calls handlers for pending data transmission requests.
void SendPendingUART(void)
{
  // Add other Tx protocols here.
  SendPendingMKSerial();
}

// -----------------------------------------------------------------------------
// This function initiates the transmission of the data in the Tx buffer.
void UARTTxBuffer(size_t tx_length)
{
  if (tx_length == 0) return;
  tx_ptr_ = &tx_buffer_[0];
  tx_bytes_remaining_ = tx_length;
  // UCSR0B |= _BV(UDRIE0);  // Enable the USART0 data register empty interrupt.
}

// -----------------------------------------------------------------------------
// This function immediately transmits a byte and blocks computation until
// transmission is commenced.
void UARTTxByte(uint8_t byte)
{
  while(UART_GetFlagStatus(UART1, UART_FLAG_TxFIFOFull));
  UART_SendData(UART1, byte);
}

// -----------------------------------------------------------------------------
// This function mimics printf, but puts the result on the UART stream. It also
// adds the end-of-line characters and checks that the character buffer is not
// exceeded. Note that this function is slow and blocking.
void UARTPrintf(const char *format, ...)
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
  while (*pointer) UARTTxByte(*pointer++);
}


// =============================================================================
// Private functions:

void ReceiveUARTData(void)
{
  VIC_ITCmd(UART1_ITLine, DISABLE);
  while (!UART_GetFlagStatus(UART1, UART_FLAG_RxFIFOEmpty))
  {
    rx_buffer_head_ = (rx_buffer_head_ + 1) % UART_RX_BUFFER_LENGTH;
    rx_buffer_[rx_buffer_head_] = UART_ReceiveData(UART1);
  }
  VIC_ITCmd(UART1_ITLine, ENABLE);
}

// -----------------------------------------------------------------------------
void UART1_IRQHandler(void)
{
  UART_ClearITPendingBit(UART1, UART_IT_Receive);
  ReceiveUARTData();
}
