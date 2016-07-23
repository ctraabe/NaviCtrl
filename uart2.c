#include "uart2.h"

#include "91x_lib.h"
#include "irq_priority.h"
#include "tr_serial_protocol.h"


// =============================================================================
// Private data:

#define UART2_BAUD (115200)

static volatile uint8_t rx_fifo_[UART2_RX_FIFO_LENGTH];
static volatile size_t rx_fifo_head_ = 0, tx_bytes_remaining_ = 0;
static uint8_t data_buffer_[UART2_DATA_BUFFER_LENGTH];


// =============================================================================
// Private function declarations:

static inline void ReceiveUARTData(void);


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
  // static enum UART2RxMode mode = UART2_RX_MODE_IDLE;

  // Process each byte.
  while (rx_fifo_tail != rx_fifo_head_)
  {
    // Move the ring buffer tail forward.
    rx_fifo_tail = (rx_fifo_tail + 1) % UART2_RX_FIFO_LENGTH;

    // Add other Rx protocols here.
    // mode = TRSerialRx(rx_fifo_[rx_fifo_tail], data_buffer_);
    TRSerialRx(rx_fifo_[rx_fifo_tail], data_buffer_);
  }
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
  ReceiveUARTData();
}


// =============================================================================
// Private functions:

static inline void ReceiveUARTData(void)
{
  while (!UART_GetFlagStatus(UART2, UART_FLAG_RxFIFOEmpty))
  {
    rx_fifo_head_ = (rx_fifo_head_ + 1) % UART2_RX_FIFO_LENGTH;
    rx_fifo_[rx_fifo_head_] = UART_ReceiveData(UART2);
  }
}
