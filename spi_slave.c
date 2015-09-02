#include "spi_slave.h"

#include "91x_lib.h"
#include "irq_priority.h"
#include "logging.h"
// TODO: Remove
#include "led.h"


// =============================================================================
// Private data:

#define SPI_START_BYTE (0xFE)

static volatile size_t rx_bytes_remaining_ = 0;
static volatile uint8_t * volatile rx_pointer_ = 0;

// volatile struct SensorData {
//   uint16_t timestamp;
//   int16_t accelerometer_sum[3];
//   int16_t gyro_sum[3];
//   uint16_t biased_pressure;
//   uint16_t battery_voltage;
// } __attribute__((packed)) sensor_data_;
static volatile char temp[17] = { 0 };
static volatile uint32_t bytes_received = 0;


// =============================================================================
// Accessors:

char * SPITemp(void)
{
  return (char *)temp;
}
uint32_t SPIBytesReceived(void)
{
  return bytes_received;
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
  SSP_DeInit(SSP0);
  SSP_Init(SSP0, &ssp_init);
  SSP_Cmd(SSP0, ENABLE);

  SSP_ITConfig(SSP0, SSP_IT_RxFifo, ENABLE);
  VIC_Config(SSP0_ITLine, VIC_IRQ, IRQ_PRIORITY_SSP0);
  VIC_ITCmd(SSP0_ITLine, ENABLE);
}


// =============================================================================
// Private functions:

void SPISlaveHandler(void)
{
  while ((rx_bytes_remaining_ == 0) &&
    SSP_GetFlagStatus(SSP0, SSP_FLAG_RxFifoNotEmpty))
  {
    ++bytes_received;
    if (SSP_ReceiveData(SSP0) == SPI_START_BYTE)
    {
      // rx_bytes_remaining_ = sizeof(struct SensorData);
      // rx_pointer_ = (volatile uint8_t * volatile)&sensor_data_;
      rx_bytes_remaining_ = 17;
      rx_pointer_ = (volatile uint8_t * volatile)temp;
    }
  }

  while ((rx_bytes_remaining_ != 0) &&
    SSP_GetFlagStatus(SSP0, SSP_FLAG_RxFifoNotEmpty))
  {
    ++bytes_received;
    *rx_pointer_++ = SSP_ReceiveData(SSP0);
    --rx_bytes_remaining_;
    if (rx_bytes_remaining_ == 0)
    {
      RedLEDOn();
      DataReadyToLog(DATA_READY_BIT_FC);
      VIC_SWITCmd(EXTIT1_ITLine, ENABLE);
    }
  }
}
