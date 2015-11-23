#include "flt_ctrl_comms.h"

#include "91x_lib.h"
#include "crc16.h"
#include "irq_priority.h"
#include "union_types.h"


// =============================================================================
// Private data:

#define SPI_FC_START_BYTE (0xAA)
#define SPI_DATA_BUFFER_LENGTH (64)

struct FromFC {
  float acceleration[3];
  float angular_rate[3];
  float quaternion[4];
} __attribute__((packed));

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
  WIU_DeInit();
  wiu_init.WIU_Line = WIU_Line16;  // Pin P6.0
  wiu_init.WIU_TriggerEdge = WIU_FallingEdge;
  WIU_Init(&wiu_init);

  WIU_ClearITPendingBit(WIU_Line16);
  WIU_Cmd(ENABLE);

  VIC_Config(WIU_ITLine, VIC_IRQ, IRQ_PRIORITY_FLT_CTRL);
  VIC_ITCmd(WIU_ITLine, ENABLE);
}

// -----------------------------------------------------------------------------
void ReleaseFltCtrlInterrupt(void)
{
  // Configure P6.0 -> FltCtrl interrupt as an input pin.
  GPIO6->DDR &= ~GPIO_Pin_0;  // Input
  SCU->GPIOOUT[6] &= ~(0x03);  // Alternate input 1
  SCU->GPIOTYPE[6] &= ~(0x1);  // Push-pull

  // Re-enable the pin change interrupt.
  WIU_ClearITPendingBit(WIU_Line16);
  VIC1->INTER |= (0x01 << (WIU_ITLine - 16));
}

// -----------------------------------------------------------------------------
void SetFltCtrlInterrupt(void)
{
  // Disable the pin change interrupt.
  VIC1->INTECR |= (0x01 << (WIU_ITLine - 16));

  // Configure P6.0 -> FltCtrl interrupt as an output pin.
  GPIO6->DDR |= GPIO_Pin_0;  // Output
  SCU->GPIOOUT[6] |= 0x01;  // Alternate output 1
  SCU->GPIOTYPE[6] |= 0x1;  // Open drain
  GPIO6->DR[GPIO_Pin_0 << 2] = 0x00;  // Set output low
}

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_buffer_) by the Rx interrupt handler.
void ProcessIncomingFltCtrlByte(uint8_t byte)
{
  static size_t bytes_processed = 0;
  static uint8_t * data_buffer_ptr = (uint8_t *)&data_buffer_[0];
  static union U16Bytes crc;
  const size_t payload_length = sizeof(struct FromFC);

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

  return;

  RESET:
  bytes_processed = 0;
}
