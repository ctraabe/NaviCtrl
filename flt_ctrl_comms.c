#include "flt_ctrl_comms.h"

#include "91x_lib.h"
#include "crc16.h"
#include "irq_priority.h"
#include "main.h"
#include "timing.h"
#include "spi_slave.h"
#include "union_types.h"


// =============================================================================
// Private data:

#define SPI_FC_START_BYTE (0xAA)

static struct FromFC {
  float acceleration[3];
  float angular_rate[3];
  float quaternion[4];
} __attribute__((packed)) from_fc_[2];

static size_t from_fc_head_ = 1, from_fc_tail_ = 0;


// =============================================================================
// Accessors:

const float * AccelerationVector(void)
{
  return from_fc_[from_fc_tail_].acceleration;
}

// -----------------------------------------------------------------------------
const float * AngularRateVector(void)
{
  return from_fc_[from_fc_tail_].angular_rate;
}

// -----------------------------------------------------------------------------
const float * Quat(void)
{
  return from_fc_[from_fc_tail_].quaternion;
}


// =============================================================================
// Public functions:

void FltCtrlCommsInit(void)
{
  SCU_APBPeriphClockConfig(__GPIO6 ,ENABLE);
  SCU_APBPeriphClockConfig(__WIU, ENABLE);

  GPIO_InitTypeDef gpio_init;

  // Configure P6.0 -> FltCtrl interrupt as an input pin.
  gpio_init.GPIO_Direction = GPIO_PinInput;
  gpio_init.GPIO_Pin = GPIO_Pin_0;
  gpio_init.GPIO_Type = GPIO_Type_PushPull;
  gpio_init.GPIO_IPInputConnected = GPIO_IPInputConnected_Disable;
  gpio_init.GPIO_Alternate = GPIO_InputAlt1;
  GPIO_Init(GPIO6, &gpio_init);

  WIU_InitTypeDef wiu_init;

  // Configure WIU to trigger an interrupt on the falling edge of pin P6.0.
  wiu_init.WIU_Line = WIU_Line16;  // Pin P6.0
  wiu_init.WIU_TriggerEdge = WIU_FallingEdge;
  WIU_Init(&wiu_init);

  WIU_ClearITPendingBit(WIU_Line16);
  SCU_WakeUpLineConfig(16);
  WIU_Cmd(ENABLE);

  VIC_Config(EXTIT2_ITLine, VIC_IRQ, IRQ_PRIORITY_FLT_CTRL);
  VIC_ITCmd(EXTIT2_ITLine, ENABLE);
}

// -----------------------------------------------------------------------------
// This function pulls the interrupt line down for about 1 microsecond.
void NotifyFltCtrl(void)
{
  // Disable the pin change interrupt.
  VIC1->INTECR |= (0x01 << (EXTIT2_ITLine - 16));

  // Configure P6.0 -> FltCtrl interrupt as an output pin.
  GPIO6->DDR |= GPIO_Pin_0;  // Output
  SCU->GPIOOUT[6] |= 0x01;  // Alternate output 1
  SCU->GPIOTYPE[6] |= 0x1;  // Open drain
  GPIO6->DR[GPIO_Pin_0 << 2] = 0x00;  // Set output low

  MicroWait(1);  // Wait 1 microsecond

  // Configure P6.0 -> FltCtrl interrupt as an input pin.
  GPIO6->DDR &= ~GPIO_Pin_0;  // Input
  SCU->GPIOOUT[6] &= ~(0x03);  // Alternate input 1
  SCU->GPIOTYPE[6] &= ~(0x1);  // Push-pull

  // Re-enable the pin change interrupt.
  WIU_ClearITPendingBit(WIU_Line16);
  VIC1->INTER |= (0x01 << (EXTIT2_ITLine - 16));
}

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_buffer_) by the Rx interrupt handler.
void ProcessIncomingFltCtrlByte(uint8_t byte)
{
  static size_t bytes_processed = 0;
  static uint8_t * from_fc_ptr = (uint8_t *)&from_fc_[0];
  static union U16Bytes crc;
  const size_t payload_length = sizeof(struct FromFC);

  switch (bytes_processed)
  {
    case 0:  // Check for start character
      if (byte != SPI_FC_START_BYTE) goto RESET;
      from_fc_ptr = (uint8_t *)&from_fc_[from_fc_head_];
      crc.u16 = 0xFFFF;
      break;
    case 1:  // Payload length
      if (byte != sizeof(struct FromFC)) goto RESET;
      crc.u16 = CRCUpdateCCITT(crc.u16, byte);
      break;
    default:  // Payload or checksum
      if (bytes_processed < (2 + payload_length))  // Payload
      {
        *from_fc_ptr++ = byte;
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
          from_fc_tail_ = from_fc_head_;
          from_fc_head_ = !from_fc_tail_;
          DataReady(DATA_READY_BIT_FC);
        }
        goto RESET;
      }
      break;
  }
  bytes_processed++;

  return;

  RESET:
  bytes_processed = 0;
}

// -----------------------------------------------------------------------------
void SendDataToFltCtrl(void)
{
  struct ToFC {
    float position[3];
    float velocity[3];
    float heading_correction;
    uint16_t crc;
  } __attribute__((packed));

  // _Static_assert(sizeof(struct ToFC) < SPI_TX_BUFFER_LENGTH,
  //   "ToFC is too large for the SPI TX buffer");

  struct ToFC * to_fc_ptr = (struct ToFC *)RequestSPITxBuffer();
  if (!to_fc_ptr) return;

  to_fc_ptr->position[0] = 0.0;
  to_fc_ptr->position[1] = 1.0;
  to_fc_ptr->position[2] = 2.0;
  to_fc_ptr->velocity[0] = 3.0;
  to_fc_ptr->velocity[1] = 4.0;
  to_fc_ptr->velocity[2] = 5.0;
  to_fc_ptr->heading_correction = 6.0;

  to_fc_ptr->crc = CRCCCITT((uint8_t *)to_fc_ptr, sizeof(struct ToFC) - 2);

  SPITxBuffer(sizeof(struct ToFC));
  NotifyFltCtrl();
}
