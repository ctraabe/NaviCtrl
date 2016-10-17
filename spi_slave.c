#include "spi_slave.h"

#include "91x_lib.h"
#include "flight_ctrl_comms.h"
#include "irq_priority.h"
#include "main.h"


// =============================================================================
// Private data:

#define SPI_RX_BUFFER_LENGTH_POWER_OF_2 (6)  // 2^6 = 64
#define SPI_RX_BUFFER_LENGTH (1 << SPI_RX_BUFFER_LENGTH_POWER_OF_2)

static volatile uint8_t rx_buffer_[SPI_RX_BUFFER_LENGTH];
static volatile size_t rx_buffer_head_ = 0, tx_bytes_remaining_ = 0;
static const uint8_t * volatile tx_ptr_ = 0;


// =============================================================================
// Public functions:

void SPISlaveInit(void)
{
  SCU_APBPeriphClockConfig(__GPIO2 ,ENABLE);
  SCU_APBPeriphClockConfig(__SSP0 ,ENABLE);

  GPIO_InitTypeDef gpio_init;

  // Configure P2.4 -> SCK, P2.5 -> MOSI, and P2.7 -> SS as inputs.
  gpio_init.GPIO_Direction = GPIO_PinInput;
  gpio_init.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7;
  gpio_init.GPIO_Type = GPIO_Type_PushPull ;
  gpio_init.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
  gpio_init.GPIO_Alternate = GPIO_InputAlt1;
  GPIO_Init(GPIO2, &gpio_init);

  // Configure P2.6 -> MISO as an input.
  gpio_init.GPIO_Direction = GPIO_PinOutput;
  gpio_init.GPIO_Pin = GPIO_Pin_6;
  gpio_init.GPIO_Alternate = GPIO_OutputAlt2;
  GPIO_Init(GPIO2, &gpio_init);

  SSP_InitTypeDef ssp_init;

  SSP_StructInit(&ssp_init);
  ssp_init.SSP_Mode = SSP_Mode_Slave;
  ssp_init.SSP_SlaveOutput = SSP_SlaveOutput_Enable;
  SSP_DeInit(SSP0);
  SSP_Init(SSP0, &ssp_init);
  SSP_Cmd(SSP0, ENABLE);

  SSP_ITConfig(SSP0, SSP_IT_RxFifo, ENABLE);
  VIC_Config(SSP0_ITLine, VIC_IRQ, IRQ_PRIORITY_SSP0);
  VIC_ITCmd(SSP0_ITLine, ENABLE);
}

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_buffer_) by the Rx interrupt handler.
void ProcessIncomingSPISlave(void)
{
  static size_t rx_buffer_tail = 0;

  while (rx_buffer_tail != rx_buffer_head_)
  {
    // Move the ring buffer tail forward.
    rx_buffer_tail = (rx_buffer_tail + 1) % SPI_RX_BUFFER_LENGTH;
    ProcessIncomingFlightCtrlByte(rx_buffer_[rx_buffer_tail]);
  }
}

// -----------------------------------------------------------------------------
void SPITxBuffer(uint8_t * buffer, uint8_t tx_length)
{
  if (tx_bytes_remaining_ != 0 || tx_length == 0) return;
  tx_ptr_ = buffer;
  tx_bytes_remaining_ = tx_length;
  while (tx_bytes_remaining_ && SSP_GetFlagStatus(SSP0, SSP_FLAG_TxFifoNotFull))
  {
    SSP_SendData(SSP0, *(tx_ptr_++));
    --tx_bytes_remaining_;
  }
}

// -----------------------------------------------------------------------------
// This interrupt gets triggered after 4 bytes have been received into the
// hardware SSP FIFO.
void SPISlaveHandler(void)
{
  while (SSP_GetFlagStatus(SSP0, SSP_FLAG_RxFifoNotEmpty))
  {
    rx_buffer_head_ = (rx_buffer_head_ + 1) % SPI_RX_BUFFER_LENGTH;
    rx_buffer_[rx_buffer_head_] = SSP_ReceiveData(SSP0);
  }

  while (tx_bytes_remaining_ && SSP_GetFlagStatus(SSP0, SSP_FLAG_TxFifoNotFull))
  {
    SSP_SendData(SSP0, *(tx_ptr_++));
    --tx_bytes_remaining_;
  }

  SetNewDataCallback(ProcessIncomingSPISlave);
}
