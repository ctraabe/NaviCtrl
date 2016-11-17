#include "uart2.h"

#include "91x_lib.h"
#include "irq_priority.h"
#include "mk_serial_protocol.h"
#include "uart.h"
#include "ut_serial_protocol.h"


// =============================================================================
// Private data:

#define UART2_BAUD (115200)

static volatile uint8_t rx_fifo_[UART_RX_FIFO_LENGTH];
static volatile size_t rx_fifo_head_ = 0, tx_bytes_remaining_ = 0;
static const uint8_t * volatile tx_ptr_ = 0;
static uint8_t data_buffer_[UART_DATA_BUFFER_LENGTH];
static uint8_t tx_buffer_[UART_TX_BUFFER_LENGTH];
static uint8_t tx_overflow_counter_ = 0;


// =============================================================================
// Private function declarations:

static inline void ReceiveUART2Data(void);
static inline void SendUART2Data(void);


// =============================================================================
// Public functions:

void UART2Init(void)
{
  SCU_APBPeriphClockConfig(__GPIO3, ENABLE);  // Enable the GPIO3 Clock
  SCU_APBPeriphClockConfig(__GPIO5, ENABLE);  // Enable the GPIO5 Clock
  SCU_APBPeriphClockConfig(__UART2, ENABLE);  // Enable the UART2 Clock

  GPIO_InitTypeDef gpio_init;

  // Configure pin GPIO5.2 to be UART2 Rx
  gpio_init.GPIO_Direction = GPIO_PinInput;
  gpio_init.GPIO_Pin = GPIO_Pin_2;
  gpio_init.GPIO_Type = GPIO_Type_PushPull;
  gpio_init.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
  gpio_init.GPIO_Alternate = GPIO_InputAlt1;  // UART2 Rx
  GPIO_Init(GPIO5, &gpio_init);

  // Configure pin GPIO3.3 to be UART2 Tx
  gpio_init.GPIO_Direction = GPIO_PinOutput;
  gpio_init.GPIO_Pin = GPIO_Pin_0;
  gpio_init.GPIO_Type = GPIO_Type_PushPull;
  gpio_init.GPIO_IPInputConnected = GPIO_IPInputConnected_Disable;
  gpio_init.GPIO_Alternate = GPIO_OutputAlt2;  // UART2 Tx
  GPIO_Init(GPIO3, &gpio_init);

  UART_InitTypeDef uart_init;

  uart_init.UART_WordLength = UART_WordLength_8D;
  uart_init.UART_StopBits = UART_StopBits_1;
  uart_init.UART_Parity = UART_Parity_No ;
  uart_init.UART_BaudRate = UART2_BAUD;
  uart_init.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
  uart_init.UART_Mode = UART_Mode_Tx_Rx;
  uart_init.UART_FIFO = UART_FIFO_Enable;
  uart_init.UART_TxFIFOLevel = UART_FIFOLevel_1_4;
  uart_init.UART_RxFIFOLevel = UART_FIFOLevel_3_4;
  UART_DeInit(UART2);
  UART_Init(UART2, &uart_init);
  UART_Cmd(UART2, ENABLE);

  // Enable UART Rx interrupt.
  UART_ITConfig(UART2, UART_IT_Receive, ENABLE);
  VIC_Config(UART2_ITLine, VIC_IRQ, IRQ_PRIORITY_UART2);
  VIC_ITCmd(UART2_ITLine, ENABLE);
}

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_fifo_) by the Rx interrupt handler. Each byte is passed to the
// appropriate Rx handler, which may place it into the temporary data buffer
// (data_buffer_).
void ProcessIncomingUART2(void)
{
  static size_t rx_fifo_tail = 0;
  static enum UARTRxMode mode = UART_RX_MODE_IDLE;

  // Make sure nothing is remaining in the UART2 hardware receive FIFO.
  VIC_ITCmd(UART2_ITLine, DISABLE);
  ReceiveUART2Data();
  VIC_ITCmd(UART2_ITLine, ENABLE);

  // Process each byte.
  while (rx_fifo_tail != rx_fifo_head_)
  {
    // Move the ring buffer tail forward.
    rx_fifo_tail = (rx_fifo_tail + 1) % UART_RX_FIFO_LENGTH;

    // Add Rx protocols here.
    switch (mode)
    {
      default:
      case UART_RX_MODE_IDLE:
        switch (rx_fifo_[rx_fifo_tail])
        {
          case MK_START_CHARACTER:
            mode = UART_RX_MODE_MK_ONGOING;
            break;
          case UT_START_CHARACTER:
            mode = UART_RX_MODE_UT_ONGOING;
            break;
          default:
            break;
        }
        break;
      case UART_RX_MODE_MK_ONGOING:
        mode = MKSerialRx(rx_fifo_[rx_fifo_tail], data_buffer_);
        break;
      case UART_RX_MODE_UT_ONGOING:
        mode = UTSerialRx(rx_fifo_[rx_fifo_tail], data_buffer_);
        break;
    }
  }
}

// -----------------------------------------------------------------------------
// This function returns the address of the shared Tx buffer (tx_buffer_) if it
// is available or zero (NULL) if not.
uint8_t * RequestUART2TxBuffer(void)
{
  if (tx_bytes_remaining_ != 0)
  {
    tx_overflow_counter_++;
    return 0;
  }
  return tx_buffer_;
}

// -----------------------------------------------------------------------------
// This function initiates the transmission of the data in the Tx buffer.
void UART2TxBuffer(size_t tx_length)
{
  if (tx_bytes_remaining_ != 0 || tx_length == 0
    || tx_length > UART_TX_BUFFER_LENGTH) return;

  tx_ptr_ = &tx_buffer_[0];
  tx_bytes_remaining_ = tx_length;

  // Fill up the UART2 hardware transmit FIFO.
  VIC_ITCmd(UART2_ITLine, DISABLE);
  SendUART2Data();
  VIC_ITCmd(UART2_ITLine, ENABLE);

  // Enable the transmit FIFO almost empty interrupt.
  if (tx_bytes_remaining_) UART_ITConfig(UART2, UART_IT_Transmit, ENABLE);
}

// -----------------------------------------------------------------------------
// This function immediately transmits a byte and blocks computation until
// transmission is commenced.
void UART2TxByte(uint8_t byte)
{
  while(UART_GetFlagStatus(UART2, UART_FLAG_TxFIFOFull));
  UART_SendData(UART2, byte);
}

// -----------------------------------------------------------------------------
void UART2Handler(void)
{
  UART_ClearITPendingBit(UART2, UART_IT_Receive);
  ReceiveUART2Data();
}


// =============================================================================
// Private functions:

static inline void ReceiveUART2Data(void)
{
  while (!UART_GetFlagStatus(UART2, UART_FLAG_RxFIFOEmpty))
  {
    rx_fifo_head_ = (rx_fifo_head_ + 1) % UART_RX_FIFO_LENGTH;
    rx_fifo_[rx_fifo_head_] = UART_ReceiveData(UART2);
  }
}

// -----------------------------------------------------------------------------
static inline void SendUART2Data(void)
{
  while (tx_bytes_remaining_ != 0
    && !UART_GetFlagStatus(UART2, UART_FLAG_TxFIFOFull))
  {
    UART_SendData(UART2, *(tx_ptr_++));
    --tx_bytes_remaining_;
  }
  if (tx_bytes_remaining_ == 0) UART_ITConfig(UART2, UART_IT_Transmit, DISABLE);
}
