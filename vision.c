#include "vision.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "91x_lib.h"
#include "attitude.h"
#include "crc16.h"
#include "irq_priority.h"
#include "quaternion.h"
#include "timing.h"
#include "union_types.h"
//TODO: remove
#include "led.h"
#include "uart.h"


// =============================================================================
// Private data:

#define VISION_UART_BAUD (57600)
#define VISION_RX_BUFFER_LENGTH (1 << 7)  // 2^7 = 128
#define VISION_FRESHNESS_LIMIT (100)  // millisends

#define VISION_START_BYTE (0xFE)

static volatile uint8_t rx_buffer_[VISION_RX_BUFFER_LENGTH];
static volatile size_t rx_buffer_head_ = 0;

static float quaternion_[4] = { 1.0, 0.0, 0.0, 0.0 };
static float heading_ = 0.0;
static uint32_t last_reception_timestamp_ = 0;
static enum VisionErrorBits vision_error_bits_ = VISION_ERROR_BIT_STALE;
// TODO: remove (this is only here for logging)
static struct FromVision from_vision_;
static float position[3] = { 0 };


// =============================================================================
// Private function declarations:

static uint32_t ProcessIncomingVisionByte(uint8_t byte);
static void ProcessVisionData(struct FromVision * from_vision);
static void ReceiveVisionData(void);


// =============================================================================
// Accessors:

float VisionHeading(void)
{
  return heading_;
}

// -----------------------------------------------------------------------------
float VisionPosition(enum WorldAxes axis)
{
  return 0.0;  // from_vision_.position[axis];
}

// -----------------------------------------------------------------------------
const float * VisionPositionVector(void)
{
  return &position[0];
}

// -----------------------------------------------------------------------------
const float * VisionQuaternionVector(void)
{
  return &quaternion_[0];
}

// -----------------------------------------------------------------------------
uint16_t VisionStatus(void)
{
  return 0;
}

// -----------------------------------------------------------------------------
uint32_t VisionTimestamp(void)
{
  return from_vision_.timestamp[0];  // from_vision_.timestamp;
}

// TODO: remove (this is only here for logging)
// -----------------------------------------------------------------------------
const struct FromVision * FromVision(void)
{
  return &from_vision_;
}


// =============================================================================
// Public functions:

void VisionInit(void)
{
  SCU_APBPeriphClockConfig(__GPIO6, ENABLE);  // Enable the GPIO6 Clock
  SCU_APBPeriphClockConfig(__UART0, ENABLE);  // Enable the UART0 Clock

  GPIO_InitTypeDef gpio_init;

  // Configure pin GPIO6.6 to be UART0 Rx
  gpio_init.GPIO_Direction = GPIO_PinInput;
  gpio_init.GPIO_Pin = GPIO_Pin_6;
  gpio_init.GPIO_Type = GPIO_Type_PushPull;
  gpio_init.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
  gpio_init.GPIO_Alternate = GPIO_InputAlt1;  // UART0 Rx
  GPIO_Init(GPIO6, &gpio_init);

  // Configure pin GPIO6.6 to be UART0 Tx
  gpio_init.GPIO_Direction = GPIO_PinOutput;
  gpio_init.GPIO_Pin = GPIO_Pin_7;
  gpio_init.GPIO_Type = GPIO_Type_PushPull;
  gpio_init.GPIO_IPInputConnected = GPIO_IPInputConnected_Disable;
  gpio_init.GPIO_Alternate = GPIO_OutputAlt3;  // UART0 Tx
  GPIO_Init(GPIO6, &gpio_init);

  UART_InitTypeDef uart_init;

  uart_init.UART_WordLength = UART_WordLength_8D;
  uart_init.UART_StopBits = UART_StopBits_1;
  uart_init.UART_Parity = UART_Parity_No ;
  uart_init.UART_BaudRate = VISION_UART_BAUD;
  uart_init.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
  uart_init.UART_Mode = UART_Mode_Tx_Rx;
  uart_init.UART_FIFO = UART_FIFO_Enable;
  uart_init.UART_TxFIFOLevel = UART_FIFOLevel_1_4;
  uart_init.UART_RxFIFOLevel = UART_FIFOLevel_3_4;
  UART_DeInit(UART0);
  UART_Init(UART0, &uart_init);
  UART_Cmd(UART0, ENABLE);

  // Enable UART Rx interrupt.
  UART_ITConfig(UART0, UART_IT_Receive, ENABLE);
  VIC_Config(UART0_ITLine, VIC_IRQ, IRQ_PRIORITY_UART0);
  VIC_ITCmd(UART0_ITLine, ENABLE);
}

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_buffer_) by the Rx interrupt handler.
uint32_t ProcessIncomingVision(void)
{
  static size_t rx_buffer_tail = 0;
  uint32_t new_data = 0;

  // Disable the UART interrupt, empty the UART Rx buffer, then re-enable.
  VIC_ITCmd(UART0_ITLine, DISABLE);
  ReceiveVisionData();
  VIC_ITCmd(UART0_ITLine, ENABLE);

  while (rx_buffer_tail != rx_buffer_head_)
  {
    // Move the ring buffer tail forward.
    rx_buffer_tail = (rx_buffer_tail + 1) % VISION_RX_BUFFER_LENGTH;
    new_data |= ProcessIncomingVisionByte(rx_buffer_[rx_buffer_tail]);
  }

  return new_data;
}

// -----------------------------------------------------------------------------
void CheckVisionFreshness(void)
{
  // Only check freshness if the data is not yet stale because the timestamp
  // might rollover, giving a false freshness.
  if ((~vision_error_bits_ & VISION_ERROR_BIT_STALE) &&
    (MillisSinceTimestamp(last_reception_timestamp_) > VISION_FRESHNESS_LIMIT))
  {
    vision_error_bits_ |= VISION_ERROR_BIT_STALE;
    // from_vision_.status = 0;
  }
}

// =============================================================================
// Private functions:

static uint32_t ProcessIncomingVisionByte(uint8_t byte)
{
#define PAYLOAD_LENGTH (sizeof(struct FromVision))
  static size_t bytes_processed = 0;
  static uint8_t payload_buffer[PAYLOAD_LENGTH];
  static uint8_t * payload_ptr = payload_buffer;
  static union U16Bytes crc;
  uint32_t new_data = 0;

  switch(bytes_processed)
  {
    case 0:  // Start byte
      if (byte != VISION_START_BYTE) goto RESET;
      payload_ptr = payload_buffer;
      crc.u16 = 0xFFFF;
      break;
    case 1:  // Payload length
      if (byte != PAYLOAD_LENGTH) goto RESET;
    case 2:  // Message ID
      crc.u16 = CRCUpdateCCITT(crc.u16, byte);
      break;
    default:  // Payload or checksum
      if (bytes_processed < (3 + PAYLOAD_LENGTH))  // Payload
      {
        *payload_ptr++ = byte;
        crc.u16 = CRCUpdateCCITT(crc.u16, byte);
      }
      else if (bytes_processed == (3 + PAYLOAD_LENGTH))  // CRC lower byte
      {
        if(byte != crc.bytes[0]) goto RESET;
      }
      else  // CRC upper byte
      {
        if(byte == crc.bytes[1])
        {
          ProcessVisionData((struct FromVision *)payload_buffer);
          // TODO: remove (this is only here for logging)
          new_data = 1;
          last_reception_timestamp_ = GetTimestamp();
          vision_error_bits_ &= ~VISION_ERROR_BIT_STALE;
        }
        goto RESET;
      }
      break;
  }
  bytes_processed++;

  return new_data;

  RESET:
  bytes_processed = 0;

  return new_data;
}

// -----------------------------------------------------------------------------
static void ProcessVisionData(struct FromVision * from_vision)
{
  memcpy(&from_vision_, from_vision, PAYLOAD_LENGTH);

  RedLEDOn();
  UARTPrintfSafe("%d", (uint16_t)(from_vision->timestamp[0] - from_vision->timestamp[1]));

  // // Compute full quaternion.
  // quaternion_[1] = from_vision->quaternion[0];
  // quaternion_[2] = from_vision->quaternion[1];
  // quaternion_[3] = from_vision->quaternion[2];
  // quaternion_[0] = sqrt(1.0 - quaternion_[1] * quaternion_[1] - quaternion_[2]
  //   * quaternion_[2] - quaternion_[3] * quaternion_[3]);

  // // Compute heading.
  // heading_ = HeadingFromQuaternion(quaternion_);
}

// -----------------------------------------------------------------------------
static void ReceiveVisionData(void)
{
  while (!UART_GetFlagStatus(UART0, UART_FLAG_RxFIFOEmpty))
  {
    rx_buffer_head_ = (rx_buffer_head_ + 1) % VISION_RX_BUFFER_LENGTH;
    rx_buffer_[rx_buffer_head_] = UART_ReceiveData(UART0);
  }
}

// -----------------------------------------------------------------------------
void VisionUARTHandler(void)
{
  UART_ClearITPendingBit(UART0, UART_IT_Receive);
  ReceiveVisionData();
}
