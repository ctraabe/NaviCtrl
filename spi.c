#include "spi.h"

#include "91x_lib.h"
#include "config.h"
// TODO: Remove
#include "uart.h"


// =============================================================================
// Private data:

#define F_SPI1 (2000000L)
#define CLOCK_RATE_SPI1 (5)

static volatile uint8_t spi1_rx_buffer_[SPI1_RX_BUFFER_LENGTH]
static volatile uint8_t *spi1_tx_ptr_ = 0;
static volatile size_t spi1_rx_buffer_head_ = 0, spi1_tx_bytes_remaining_ = 0;
static uint8_t spi1_tx_buffer_[UART_TX_BUFFER_LENGTH];
static uint8_t spi1_tx_overflow_counter_ = 0;


// =============================================================================
// Private function declarations:

static uint8_t SPIClockPrescaler(uint32_t target_frequency, uint8_t clock_rate);


// =============================================================================
// Accessors


// =============================================================================
// Public functions:

void SPIInit(void)
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

  UARTPrintf("SPI1 clock prescaler: %u", ssp_init.SSP_ClockPrescaler);

  SSP_ITConfig(SSP1, SSP_IT_RxFifo, ENABLE);
  VIC_Config(SSP1_ITLine, VIC_IRQ, PRIORITY_SSP1);
  VIC_ITCmd(SSP1_ITLine, ENABLE);
}

// -----------------------------------------------------------------------------
// This function initiates the transmission of the data in a Tx buffer.
void SPITxBuffer(uint8_t tx_length)
{
  if (tx_length == 0) return;
  spi1_tx_ptr_ = &spi1_tx_buffer_[0];
  spi1_tx_bytes_remaining_ = tx_length;
  SSP_ITConfig(SSP1, SSP_IT_TxFifo, ENABLE);
}

// -----------------------------------------------------------------------------
uint32_t SPIXWaitUntilCompletion(uint32_t time_limit_ms)
{
  uint32_t timeout = GetTimestampMillisFromNow(time_limit_ms);
  while (SSP_GetFlagStatus(SSP_FLAG_Busy) && !TimestampInPast(timeout))
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
  while (SSP_GetFlagStatus(SSP_FLAG_TxFifoNotFull)
    && spi1_tx_bytes_remaining_--) SSP_SendData(SSP1, *spi1_tx_ptr_++);

  while (SSP_GetFlagStatus(SSP_FLAG_RxFifoNotEmpty))
  {
    spi1_rx_buffer_head_ = (spi1_rx_buffer_head_ + 1) % SPI1_RX_BUFFER_LENGTH;
    *spi1_rx_buffer_[spi1_rx_buffer_head_] = SSP_ReceiveData(SSP1);
  }

  if (spi1_tx_length_ == 0)
  {
    SSP_ITConfig(SSP1, SSP_IT_TxFifo, DISABLE);
    // TODO: callback();
  }
}
