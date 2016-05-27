#include "vision.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "91x_lib.h"
#include "attitude.h"
#include "crc16.h"
#include "irq_priority.h"
#include "quaternion.h"
#include "union_types.h"


// =============================================================================
// Private data:

#define VISION_UART_BAUD (115200)
#define VISION_RX_BUFFER_LENGTH (1 << 7)  // 2^7 = 128

#define VISION_START_BYTE (0xFE)

static volatile uint8_t rx_buffer_[VISION_RX_BUFFER_LENGTH];
static volatile size_t rx_buffer_head_ = 0;

static float position_[3] = { 0.0 }, quaternion_[4] = { 1.0, 0.0, 0.0, 0.0 };
static float body_velocity_[3] = { 0.0 }, inertial_velocity_[3] = { 0.0 };
static float dt_ = 0.0 , heading_ = 0.0;
static uint16_t status_ = 0;
// TODO: remove (this is only here for logging)
static struct FromVision from_vision_;


// =============================================================================
// Private function declarations:

static uint32_t ProcessIncomingVisionByte(uint8_t byte);
static void ProcessVisionData(struct FromVision * from_vision);
static void ReceiveVisionData(void);


// =============================================================================
// Accessors:

const float * VisionBodyVelocityVector(void)
{
  return &body_velocity_[0];
}

// -----------------------------------------------------------------------------
float VisionDT(void)
{
  return dt_;
}

// -----------------------------------------------------------------------------
float VisionHeading(void)
{
  return heading_;
}

// -----------------------------------------------------------------------------
float VisionPosition(enum WorldAxes axis)
{
  return position_[axis];
}

// -----------------------------------------------------------------------------
const float * VisionPositionVector(void)
{
  return &position_[0];
}

// -----------------------------------------------------------------------------
const float * VisionQuaternionVector(void)
{
  return &quaternion_[0];
}

// -----------------------------------------------------------------------------
uint16_t VisionStatus(void)
{
  return status_;
}

// -----------------------------------------------------------------------------
const float * VisionVelocityVector(void)
{
  return &inertial_velocity_[0];
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
          memcpy(&from_vision_, payload_buffer, PAYLOAD_LENGTH);
          new_data = 1;
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
  static float position_pv[3] = { 0.0 };
  // Copy the position.
  position_[0] = from_vision->position[0];
  position_[1] = from_vision->position[1];
  position_[2] = from_vision->position[2];

  // Compute full quaternion.
  quaternion_[1] = from_vision->quaternion[0];
  quaternion_[2] = from_vision->quaternion[1];
  quaternion_[3] = from_vision->quaternion[2];
  quaternion_[0] = sqrt(1.0 - quaternion_[1] * quaternion_[1] - quaternion_[2]
    * quaternion_[2] - quaternion_[3] * quaternion_[3]);

  // Compute heading.
  heading_ = HeadingFromQuaternion(quaternion_);

  // Take the derivative of position.
  float dt_inv = 1.0e6 / (float)from_vision->dt;  // seconds
  inertial_velocity_[0] = (from_vision->position[0] - position_pv[0]) * dt_inv;
  inertial_velocity_[1] = (from_vision->position[1] - position_pv[1]) * dt_inv;
  inertial_velocity_[2] = (from_vision->position[2] - position_pv[2]) * dt_inv;

  // Rotate velocity to the body axis.
  QuaternionInverseRotateVector(quaternion_, inertial_velocity_,
    body_velocity_);

  status_ = from_vision->status;
  dt_ = 1e-6 * (float)from_vision->dt;
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
