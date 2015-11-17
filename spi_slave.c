#include "spi_slave.h"

#include <stdio.h>

#include "91x_lib.h"
#include "crc16.h"
#include "irq_priority.h"
#include "union_types.h"


// =============================================================================
// Private data:

#define SPI_RX_BUFFER_LENGTH_POWER_OF_2 (8)  // 2^8 = 256
#define SPI_RX_BUFFER_LENGTH (1 << SPI_RX_BUFFER_LENGTH_POWER_OF_2)
#define SPI_FC_START_BYTE (0xAA)
#define SPI_DATA_BUFFER_LENGTH (64)

struct FromFC {
  float acceleration[3];
  float angular_rate[3];
  float quaternion[4];
} __attribute__((packed));

static volatile uint8_t rx_buffer_[SPI_RX_BUFFER_LENGTH];
static volatile size_t rx_buffer_head_ = 0;
static struct FromFC data_buffer_[2];
static size_t data_buffer_head_ = 0, data_buffer_tail_ = 0;


// =============================================================================
// Accessors:

const float * AccelerationVector(void)
{
  return data_buffer_[data_buffer_tail_].acceleration;
}

// -----------------------------------------------------------------------------
const float * AngularRateVector(void)
{
  return data_buffer_[data_buffer_tail_].angular_rate;
}

// -----------------------------------------------------------------------------
const float * Quat(void)
{
  return data_buffer_[data_buffer_tail_].quaternion;
}


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
  static size_t bytes_processed = 0, rx_buffer_tail = 0;
  static uint8_t * data_buffer_ptr = (uint8_t *)&data_buffer_[0];
  static union U16Bytes crc;
  const size_t payload_length = sizeof(struct FromFC);

  while (rx_buffer_tail != rx_buffer_head_)
  {
    // Move the ring buffer tail forward.
    rx_buffer_tail = (rx_buffer_tail + 1) % SPI_RX_BUFFER_LENGTH;
    uint8_t byte = rx_buffer_[rx_buffer_tail];
    switch (bytes_processed)
    {
      case 0:  // Check for start character
        if (byte != SPI_FC_START_BYTE) goto RESET;
        data_buffer_ptr = (uint8_t *)&data_buffer_[data_buffer_head_];
        crc.u16 = 0xFFFF;
        break;
      case 1:  // Payload length
        if (byte != sizeof(struct FromFC)) goto RESET;
        crc.u16 = CRCUpdateCCITT(crc.u16, byte);
        break;
    default:  // Payload or checksum
      if (bytes_processed < (2 + payload_length))  // Payload
      {
        *data_buffer_ptr++ = byte;
        crc.u16 = CRCUpdateCCITT(crc.u16, byte);
      }
      else if (bytes_processed == (2 + payload_length))  // CRC lower byte
      {
        if (byte != crc.bytes[0]) goto RESET;
      }
      else  // CRC upper byte
      {
        if (byte == crc.bytes[1])
        {
          // Swap data buffers.
          data_buffer_tail_ = data_buffer_head_;
          data_buffer_head_ = !data_buffer_tail_;
        }
        goto RESET;
      }
      break;
    }
    bytes_processed++;
  }
  return;

  RESET:
  bytes_processed = 0;
}

// -----------------------------------------------------------------------------
void SPISlaveHandler(void)
{
  while (SSP_GetFlagStatus(SSP0, SSP_FLAG_RxFifoNotEmpty))
  {
    rx_buffer_head_ = (rx_buffer_head_ + 1) % SPI_RX_BUFFER_LENGTH;
    rx_buffer_[rx_buffer_head_] = SSP_ReceiveData(SSP0);
    // SSP_SendData(SSP0, 0);
  }
}
