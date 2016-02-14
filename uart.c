#include "uart.h"

#include <stdarg.h>
#include <stdio.h>

#include "91x_lib.h"
#include "irq_priority.h"
#include "mk_serial_protocol.h"
#include "mk_serial_tx.h"
#include "timing.h"


// =============================================================================
// Private data:

#define UART_BAUD (57600)

static volatile uint8_t rx_fifo_[UART_RX_FIFO_LENGTH];
static volatile size_t rx_fifo_head_ = 0, tx_bytes_remaining_ = 0;
static const uint8_t * volatile tx_ptr_ = 0;
static uint8_t data_buffer_[UART_DATA_BUFFER_LENGTH];
static uint8_t tx_buffer_[UART_TX_BUFFER_LENGTH];
static uint8_t tx_overflow_counter_ = 0;


// =============================================================================
// Private function declarations:

static void Printf(const char *format, va_list arglist);
static inline void ReceiveUARTData(void);
static inline void SendUARTData(void);


// =============================================================================
// Public functions:

void UARTInit(void)
{
  SCU_APBPeriphClockConfig(__GPIO3, ENABLE);  // Enable the GPIO3 Clock
  SCU_APBPeriphClockConfig(__UART1, ENABLE);  // Enable the UART1 Clock

  GPIO_InitTypeDef gpio_init;

  // Configure pin GPIO3.2 to be UART1 Rx
  gpio_init.GPIO_Direction = GPIO_PinInput;
  gpio_init.GPIO_Pin = GPIO_Pin_2;
  gpio_init.GPIO_Type = GPIO_Type_PushPull;
  gpio_init.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
  gpio_init.GPIO_Alternate = GPIO_InputAlt1;  // UART1 Rx
  GPIO_Init(GPIO3, &gpio_init);

  // Configure pin GPIO3.3 to be UART1 Tx
  gpio_init.GPIO_Direction = GPIO_PinOutput;
  gpio_init.GPIO_Pin = GPIO_Pin_3;
  gpio_init.GPIO_Type = GPIO_Type_PushPull;
  gpio_init.GPIO_Alternate = GPIO_OutputAlt2;  // UART1 Tx
  GPIO_Init(GPIO3, &gpio_init);

  UART_InitTypeDef uart_init;

  uart_init.UART_WordLength = UART_WordLength_8D;
  uart_init.UART_StopBits = UART_StopBits_1;
  uart_init.UART_Parity = UART_Parity_No ;
  uart_init.UART_BaudRate = UART_BAUD;
  uart_init.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
  uart_init.UART_Mode = UART_Mode_Tx_Rx;
  uart_init.UART_FIFO = UART_FIFO_Enable;
  uart_init.UART_TxFIFOLevel = UART_FIFOLevel_1_4;
  uart_init.UART_RxFIFOLevel = UART_FIFOLevel_3_4;
  UART_DeInit(UART1);
  UART_Init(UART1, &uart_init);
  UART_Cmd(UART1, ENABLE);

  // Enable UART Rx interrupt.
  UART_ITConfig(UART1, UART_IT_Receive, ENABLE);
  VIC_Config(UART1_ITLine, VIC_IRQ, IRQ_PRIORITY_UART1);
  VIC_ITCmd(UART1_ITLine, ENABLE);
}

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_fifo_) by the Rx interrupt handler. Each byte is passed to the
// appropriate Rx handler, which may place it into the temporary data buffer
// (data_buffer_).
void ProcessIncomingUART(void)
{
  static size_t rx_fifo_tail = 0;
  static enum UARTRxMode mode = UART_RX_MODE_IDLE;

  // Make sure nothing is remaining in the UART1 hardware receive FIFO.
  VIC_ITCmd(UART1_ITLine, DISABLE);
  ReceiveUARTData();
  VIC_ITCmd(UART1_ITLine, ENABLE);

  // Process each byte.
  while (rx_fifo_tail != rx_fifo_head_)
  {
    // Move the ring buffer tail forward.
    rx_fifo_tail = (rx_fifo_tail + 1) % UART_RX_FIFO_LENGTH;

    // Add other Rx protocols here.
    if (mode != UART_RX_MODE_IDLE)
      mode = MKSerialRx(rx_fifo_[rx_fifo_tail], data_buffer_);
    else if (rx_fifo_[rx_fifo_tail] == '#')  // MK protocol start character
      mode = UART_RX_MODE_MK_ONGOING;
  }
}

// -----------------------------------------------------------------------------
// This function returns the address of the shared Tx buffer (tx_buffer_) if it
// is available or zero (NULL) if not.
uint8_t * RequestUARTTxBuffer(void)
{
  if (tx_bytes_remaining_ != 0)
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
  if (tx_bytes_remaining_ != 0 || tx_length == 0
    || tx_length > UART_TX_BUFFER_LENGTH) return;

  tx_ptr_ = &tx_buffer_[0];
  tx_bytes_remaining_ = tx_length;

  // Fill up the UART1 hardware transmit FIFO.
  VIC_ITCmd(UART1_ITLine, DISABLE);
  SendUARTData();
  VIC_ITCmd(UART1_ITLine, ENABLE);

  // Enable the transmit FIFO almost empty interrupt.
  if (tx_bytes_remaining_) UART_ITConfig(UART1, UART_IT_Transmit, ENABLE);
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
// exceeded. This version blocks program execution until UART is available and
// then further blocks execution until the transmission has competed.
void UARTPrintf(const char *format, ...)
{
  UARTWaitUntilCompletion(500);
  va_list arglist;
  va_start(arglist, format);
  Printf(format, arglist);
  va_end(arglist);
  UARTWaitUntilCompletion(500);
}

// -----------------------------------------------------------------------------
// This function mimics printf, but puts the result on the UART stream. It also
// adds the end-of-line characters and checks that the character buffer is not
// exceeded. This version attempts to get the UART Tx buffer and then initiates
// an interrupt-bases transmission. This function is non-blocking, but may fail
// to get access to the UART Tx buffer.
void UARTPrintfSafe(const char *format, ...)
{
  va_list arglist;
  va_start(arglist, format);
  Printf(format, arglist);
  va_end(arglist);
}

// -----------------------------------------------------------------------------
uint32_t UARTWaitUntilCompletion(uint32_t time_limit_ms)
{
  uint32_t timeout = GetTimestampMillisFromNow(time_limit_ms);
  while ((tx_bytes_remaining_ != 0) && !TimestampInPast(timeout)) continue;
  return TimestampInPast(timeout);
}

// -----------------------------------------------------------------------------
void UARTHandler(void)
{
  UART_ClearITPendingBit(UART1, UART_IT_Receive);
  ReceiveUARTData();
  SendUARTData();
}


// =============================================================================
// Private functions:

// This function mimics printf, but puts the result on the UART stream. It also
// adds the end-of-line characters and checks that the character buffer is not
// exceeded.
static void Printf(const char *format, va_list arglist)
{
  // Buffer requirement: 100 chars + 2 newline chars + 1 null terminator
  _Static_assert(UART_TX_BUFFER_LENGTH >= 103,
    "UART buffer not large enough for UARTPrintf");

  uint8_t * ascii = RequestUARTTxBuffer();
  if (!ascii) return;

  int length = vsnprintf((char *)ascii, 101, (char *)format, arglist);

  if (length < 101)
  {
    sprintf((char *)&ascii[length], "\n\r");
    length += 2;
  }
  else
  {
    sprintf((char *)&ascii[80], "... MESSAGE TOO LONG\n\r");
    length = 103;
  }

  UARTTxBuffer(length);
}

// -----------------------------------------------------------------------------
static inline void ReceiveUARTData(void)
{
  while (!UART_GetFlagStatus(UART1, UART_FLAG_RxFIFOEmpty))
  {
    rx_fifo_head_ = (rx_fifo_head_ + 1) % UART_RX_FIFO_LENGTH;
    rx_fifo_[rx_fifo_head_] = UART_ReceiveData(UART1);
  }
}

// -----------------------------------------------------------------------------
static inline void SendUARTData(void)
{
  while (tx_bytes_remaining_ != 0
    && !UART_GetFlagStatus(UART1, UART_FLAG_TxFIFOFull))
  {
    UART_SendData(UART1, *(tx_ptr_++));
    --tx_bytes_remaining_;
  }
  if (tx_bytes_remaining_ == 0) UART_ITConfig(UART1, UART_IT_Transmit, DISABLE);
}
