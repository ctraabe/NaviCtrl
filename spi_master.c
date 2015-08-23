#include "spi_master.h"

#include "91x_lib.h"
#include "config.h"
#include "timing.h"
// TODO: Remove
#include "uart.h"


// =============================================================================
// Private data:

#define F_SPI1 (2000000L)
#define CLOCK_RATE_SPI1 (5)
#define SPI_MASTER_RX_FIFO_LENGTH (1 << 7)  // 2^7 = 128

static volatile uint8_t rx_fifo_[SPI_MASTER_RX_FIFO_LENGTH], * tx_ptr_ = 0;
static volatile size_t rx_bytes_remaining_ = 0, rx_fifo_head_ = 0;
static volatile size_t tx_bytes_remaining_ = 0;
static volatile size_t bytes_remaining_ = 0;
static size_t rx_fifo_tail_ = 0;
static uint8_t tx_buffer_[SPI_MASTER_TX_BUFFER_LENGTH];
static uint8_t tx_overflow_counter_ = 0;


// =============================================================================
// Private function declarations:

static uint8_t SPIClockPrescaler(uint32_t target_frequency, uint8_t clock_rate);


// =============================================================================
// Accessors


// =============================================================================
// Public functions:

void SPIMasterInit(void)
{
  SCU_APBPeriphClockConfig(__GPIO3 ,ENABLE);
  SCU_APBPeriphClockConfig(__SSP1 ,ENABLE);

  GPIO_InitTypeDef gpio_init;

  // Configure P3.5 <- MISO1 as an input pin.
  gpio_init.GPIO_Direction = GPIO_PinInput;
  gpio_init.GPIO_Pin = GPIO_Pin_5;
  gpio_init.GPIO_Type = GPIO_Type_PushPull;
  gpio_init.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
  gpio_init.GPIO_Alternate = GPIO_InputAlt1;
  GPIO_Init (GPIO3, &gpio_init);

  // Configure P3.4 -> SCK1 and P3.6 -> MOSI1 as output pins.
  gpio_init.GPIO_Direction = GPIO_PinOutput;
  gpio_init.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
  gpio_init.GPIO_Alternate = GPIO_OutputAlt2;
  GPIO_Init (GPIO3, &gpio_init);

  SSP_InitTypeDef ssp_init;

  SSP_StructInit(&ssp_init);
  ssp_init.SSP_ClockRate = CLOCK_RATE_SPI1;
  ssp_init.SSP_ClockPrescaler = SPIClockPrescaler(F_SPI1, CLOCK_RATE_SPI1);
  SSP_DeInit(SSP1);
  SSP_Init(SSP1, &ssp_init);
  SSP_Cmd(SSP1, ENABLE);

  SSP_ITConfig(SSP1, SSP_IT_RxFifo, ENABLE);
  VIC_Config(SSP1_ITLine, VIC_IRQ, PRIORITY_SSP1);
  VIC_ITCmd(SSP1_ITLine, ENABLE);
}

// -----------------------------------------------------------------------------
uint32_t SPIMasterGetByte(uint8_t * byte)
{
  if (rx_fifo_head_ == rx_fifo_tail_) return 1;

  *byte = rx_fifo_[rx_fifo_tail_];
  rx_fifo_tail_ = (rx_fifo_tail_ + 1) % SPI_MASTER_RX_FIFO_LENGTH;
  return 0;
}

// -----------------------------------------------------------------------------
// This function returns the address of the shared Tx buffer (tx_buffer_) if it
// is available or zero (NULL) if not.
uint8_t * RequestSPIMasterTxBuffer(void)
{
  if (tx_bytes_remaining_)
  {
    tx_overflow_counter_++;
    return 0;
  }
  return tx_buffer_;
}

// -----------------------------------------------------------------------------
// This function essentially flushes rx_fifo_.
void SPIMasterResetRxFIFO(void)
{
  rx_fifo_tail_ = rx_fifo_head_;
}

// -----------------------------------------------------------------------------
// This function initiates the transmission of the data in a tx_buffer_ and/or
// data reception. An number of bytes exchanged will be the maximum of rx_length
// and tx_length. The transmission will start with tx_length bytes from
// tx_buffer_ and will continue with null (0xFF) bytes if tx_length is less than
// rx_length. For reception, the first rx_length bytes will be put into rx_fifo_
// and subsequent bytes will be ignored if rx_length is less than tx_length.
void SPIMasterStart(uint8_t rx_length, uint8_t tx_length)
{
  if ((rx_length == 0) && (tx_length == 0)) return;
  tx_ptr_ = &tx_buffer_[0];
  tx_bytes_remaining_ = tx_length;
  rx_bytes_remaining_ = rx_length;
  bytes_remaining_ = rx_length > tx_length ? rx_length : tx_length;
  SSP_ITConfig(SSP1, SSP_IT_TxFifo, ENABLE);
}

// -----------------------------------------------------------------------------
uint32_t SPIMasterWaitUntilCompletion(uint32_t time_limit_ms)
{
  uint32_t timeout = GetTimestampMillisFromNow(time_limit_ms);
  while (SSP_GetFlagStatus(SSP1, SSP_FLAG_Busy) && !TimestampInPast(timeout))
    continue;
  return TimestampInPast(timeout);
}


// =============================================================================
// Private functions:

static uint8_t SPIClockPrescaler(uint32_t target_frequency, uint8_t clock_rate)
{
  uint32_t baud_rate_clock = SCU_GetMCLKFreqValue() * 1000;
  if (!(SCU->CLKCNTR & SCU_BRCLK_Div1)) baud_rate_clock /= 2;
  return (uint8_t)(baud_rate_clock / (clock_rate + 1) / target_frequency);
}

// -----------------------------------------------------------------------------
void SSP1_IRQHandler(void)
{
  while (SSP_GetFlagStatus(SSP1, SSP_FLAG_RxFifoNotEmpty))
  {
    if (rx_bytes_remaining_ == 0)
    {
      SSP_ReceiveData(SSP1);
    }
    else
    {
      rx_fifo_head_ = (rx_fifo_head_ + 1) % SPI_MASTER_RX_FIFO_LENGTH;
      rx_fifo_[rx_fifo_head_] = SSP_ReceiveData(SSP1);
      rx_bytes_remaining_--;
    }
  }

  while (SSP_GetFlagStatus(SSP1, SSP_FLAG_TxFifoNotFull) && bytes_remaining_--)
  {
    if (tx_bytes_remaining_ == 0)
    {
      SSP_SendData(SSP1, 0xFF);
    }
    else
    {
      SSP_SendData(SSP1, *tx_ptr_++);
      tx_bytes_remaining_--;
    }
  }

  if (bytes_remaining_ == 0)
  {
    SSP_ITConfig(SSP1, SSP_IT_TxFifo, DISABLE);
    // TODO: callback();
  }
}
