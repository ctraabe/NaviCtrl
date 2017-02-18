#include "flight_ctrl_comms.h"

#include "91x_lib.h"
#include "crc16.h"
#include "heading.h"
#include "irq_priority.h"
#include "kalman_filter.h"
#include "lsm303dl.h"
#include "main.h"
#include "quaternion.h"
#include "spi_slave.h"
#include "timing.h"
#include "ublox.h"
#include "union_types.h"
#include "vision.h"
#ifdef LOG_DEBUG_TO_SD
  #include "logging.h"
#endif
// TODO: remove
#include "attitude.h"
#include "custom_math.h"
#include <math.h>


// =============================================================================
// Private data:

#define NAV_COMMS_VERSION (1)
#define SPI_FC_START_BYTE (0xAA)

enum NavModeBits {
  NAV_BIT_MODE_0     = 1<<0,
  NAV_BIT_MODE_1     = 1<<1,
  NAV_BIT_HOLD_RESET = 1<<2,
  NAV_BIT_RESERVED_0 = 1<<3,
  NAV_BIT_ROUTE_0    = 1<<4,
  NAV_BIT_ROUTE_1    = 1<<5,
  NAV_BIT_SWITCH_0   = 1<<6,
  NAV_BIT_SWITCH_1   = 1<<7,
};

enum ToFlightCtrlPendingUpdateBits {
  PENDING_UPDATE_BIT_HEADING_CORRECTION = 1<<0,
  PENDING_UPDATE_BIT_NAVIGATION         = 1<<1,
  PENDING_UPDATE_BIT_POSITION           = 1<<2,
  PENDING_UPDATE_BIT_VELOCITY           = 1<<3,
};

static volatile uint32_t comms_ongoing_ = 0;

#ifdef BARO_ALTIMETER_VERTICAL_NAVIGATION
static struct ToFlightCtrl to_fc_ = { .version = NAV_COMMS_VERSION,
  .status = NAV_STATUS_BIT_LOW_PRECISION_VERTICAL };
#else
static struct ToFlightCtrl to_fc_ = { .version = NAV_COMMS_VERSION };
#endif
static struct ToFlightCtrl to_fc_buffer_ = { 0 };
static struct FromFlightCtrl from_fc_[2] = { { 0 } };
static union U16Bytes crc_[2];
static size_t from_fc_head_ = 1, from_fc_tail_ = 0;
static uint32_t new_data_ = 0, pending_update_bits_ = 0;


// =============================================================================
// Private function declarations:

static void CopyPendingToFlightControlData(void);
static void UpdateCRCToFlightCtrl(void);


// =============================================================================
// Accessors:

float Accelerometer(enum BodyAxes axis)
{
  return from_fc_[from_fc_tail_].accelerometer[axis];
}

// -----------------------------------------------------------------------------
const float * AccelerometerVector(void)
{
  return from_fc_[from_fc_tail_].accelerometer;
}

// -----------------------------------------------------------------------------
uint32_t FlightCtrlCommsOngoing(void)
{
  return comms_ongoing_;
}

// -----------------------------------------------------------------------------
uint8_t FlightCtrlState(void)
{
  return from_fc_[from_fc_tail_].state;
}

// -----------------------------------------------------------------------------
uint16_t FlightCtrlTimestamp(void)
{
  return from_fc_[from_fc_tail_].timestamp;
}

// -----------------------------------------------------------------------------
float Gyro(enum BodyAxes axis)
{
  return from_fc_[from_fc_tail_].gyro[axis];
}

// -----------------------------------------------------------------------------
const float * GyroVector(void)
{
  return from_fc_[from_fc_tail_].gyro;
}

// -----------------------------------------------------------------------------
uint32_t NewDataFromFlightCtrl(void)
{
  return new_data_;
}

// -----------------------------------------------------------------------------
const float * Quat(void)
{
  return from_fc_[from_fc_tail_].quaternion;
}

// -----------------------------------------------------------------------------
// TODO: consider making a separate "position" program unit to handle current
// position estimates
const float * PositionVector(void)
{
  return to_fc_.position;
}

// -----------------------------------------------------------------------------
float PressureAltitude(void)
{
  return from_fc_[from_fc_tail_].pressure_altitude;
}

// -----------------------------------------------------------------------------
uint32_t RCSwitch(void)
{
  return (uint32_t)(from_fc_[from_fc_tail_].nav_mode_request >> 6) & 0x03;
}

// -----------------------------------------------------------------------------
enum NavMode RequestedNavMode(void)
{
  return (enum NavMode)(from_fc_[from_fc_tail_].nav_mode_request & 0x03);
}

// -----------------------------------------------------------------------------
uint32_t RequestedNavRoute(void)
{
  return (uint32_t)(from_fc_[from_fc_tail_].nav_mode_request >> 4) & 0x03;
}

// -----------------------------------------------------------------------------
const struct FromFlightCtrl * FromFlightCtrl(void)
{
  return &from_fc_[from_fc_tail_];
}

// -----------------------------------------------------------------------------
uint16_t FromFlightCtrlCRC(void)
{
  return crc_[from_fc_tail_].u16;
}


// =============================================================================
// Public functions:

void ClearNewDataFromFlightCtrlFlag(void)
{
  new_data_ = 0;
}

// -----------------------------------------------------------------------------
void FlightCtrlCommsInit(void)
{
  SCU_APBPeriphClockConfig(__GPIO6 ,ENABLE);
  SCU_APBPeriphClockConfig(__WIU, ENABLE);

  GPIO_InitTypeDef gpio_init;

  // Configure P6.0 -> FlightCtrl interrupt as an input pin.
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

  from_fc_[0].accelerometer[2] = -1.0;
  from_fc_[0].quaternion[0] = 1.0;
}

// -----------------------------------------------------------------------------
// This function pulls the interrupt line down for about 1 microsecond.
void NotifyFlightCtrl(void)
{
  // Disable the pin change interrupt.
  VIC1->INTECR |= (0x01 << (EXTIT2_ITLine - 16));

  // Configure P6.0 -> FlightCtrl interrupt as an output pin.
  GPIO6->DDR |= GPIO_Pin_0;  // Output
  SCU->GPIOOUT[6] |= 0x01;  // Alternate output 1
  SCU->GPIOTYPE[6] |= 0x1;  // Open drain
  GPIO6->DR[GPIO_Pin_0 << 2] = 0x00;  // Set output low

  MicroWait(1);  // Wait 1 microsecond

  // Configure P6.0 -> FlightCtrl interrupt as an input pin.
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
void ProcessIncomingFlightCtrlByte(uint8_t byte)
{
  static size_t bytes_processed = 0;
  static uint8_t * from_fc_ptr = (uint8_t *)&from_fc_[0];
  const size_t payload_length = sizeof(struct FromFlightCtrl);

  switch (bytes_processed)
  {
    case 0:  // Check for start character
      if (byte != SPI_FC_START_BYTE) goto RESET;
      from_fc_ptr = (uint8_t *)&from_fc_[from_fc_head_];
      crc_[from_fc_head_].u16 = 0xFFFF;
      comms_ongoing_ = 1;
      break;
    case 1:  // Payload length
      if (byte != sizeof(struct FromFlightCtrl)) goto RESET;
      crc_[from_fc_head_].u16 = CRCUpdateCCITT(crc_[from_fc_head_].u16, byte);
      break;
    default:  // Payload or checksum
      *from_fc_ptr++ = byte;
      if (bytes_processed < (2 + payload_length))  // Payload
      {
        crc_[from_fc_head_].u16 = CRCUpdateCCITT(crc_[from_fc_head_].u16, byte);
      }
      else if (bytes_processed == (2 + payload_length))  // CRC lower byte
      {
        if (byte != crc_[from_fc_head_].bytes[0]) goto RESET;
      }
      else  // CRC upper byte
      {
        if (byte == crc_[from_fc_head_].bytes[1])
        {
          // Swap data buffers.
          from_fc_tail_ = from_fc_head_;
          from_fc_head_ = !from_fc_tail_;

          // TODO: this shouldn't depend on successful reception, but also
          // shouldn't be done prematurely. How can we tell when the
          // transmission is over? Slave select?
          CopyPendingToFlightControlData();

          new_data_ = 1;

#ifdef LOG_DEBUG_TO_SD
          LogFromFlightCtrlData();
#endif
        }
        goto RESET;
      }
      break;
  }
  bytes_processed++;

  return;

  RESET:
  bytes_processed = 0;
  comms_ongoing_ = 0;
}

// -----------------------------------------------------------------------------
void PrepareFlightCtrlDataExchange(void)
{
  SetFlightCtrlCommsOngoingFlag();
#ifdef LOG_DEBUG_TO_SD
  // LogToFlightCtrlData(to_fc_ptr);
#endif
  SPITxBuffer((uint8_t *)&to_fc_, sizeof(to_fc_));
  NotifyFlightCtrl();  // Request SPI communication.
}

// -----------------------------------------------------------------------------
void SetFlightCtrlCommsOngoingFlag(void)
{
  comms_ongoing_ = 1;
}

// -----------------------------------------------------------------------------
void UpdateHeadingCorrectionToFlightCtrl(enum Sensors sensor)
{
  // Form a heading correction
  float heading_error;
  uint32_t status;

  if ((sensor == LSM303DL) && (ActiveNavSensorBits() & SENSOR_BIT_LSM303DL))
  {
    float mag_earth[3];
    QuaternionRotateVector((float *)Quat(), MagneticVector(), mag_earth);
    // TODO: dedeclinate
    // TODO: do some sanity checking on the magnetic scale, etc.
    heading_error = -atan2(mag_earth[1], mag_earth[0]);
    status = !LSM303DLDataStale();
  }
  else if ((sensor == VISION) && (ActiveNavSensorBits() & SENSOR_BIT_VISION))
  {
    heading_error = VisionHeading() - CurrentHeading();
    status = VisionStatus();
  }
  else
  {
    return;
  }

  if (status != 1) heading_error = 0;
  WrapToPlusMinusPi(heading_error);
  // TODO: put these magic numbers in a #define
  float quat_c_z = 0.5 * 0.025 * heading_error;

  struct ToFlightCtrl * to_fc = &to_fc_;
  if (comms_ongoing_)
  {
    to_fc = &to_fc_buffer_;
    pending_update_bits_ |= PENDING_UPDATE_BIT_HEADING_CORRECTION;
  }

  to_fc->heading_correction_quat_0 = sqrt(1.0 - quat_c_z * quat_c_z);
  to_fc->heading_correction_quat_z = quat_c_z;

  if (status)
    to_fc->status |= NAV_STATUS_BIT_HEADING_DATA_OK;
  else
    to_fc->status &= ~NAV_STATUS_BIT_HEADING_DATA_OK;

  if (to_fc == &to_fc_) UpdateCRCToFlightCtrl();
}

// -----------------------------------------------------------------------------
void UpdateNavigationToFlightCtrl(void)
{
  struct ToFlightCtrl * to_fc = &to_fc_;
  if (comms_ongoing_)
  {
    to_fc = &to_fc_buffer_;
    pending_update_bits_ |= PENDING_UPDATE_BIT_NAVIGATION;
  }

  if (ActiveNavSensorBits() & SENSOR_BIT_UBLOX)
    to_fc->status |= NAV_STATUS_BIT_LOW_PRECISION_VERTICAL;
  else
    to_fc->status &= ~NAV_STATUS_BIT_LOW_PRECISION_VERTICAL;

  to_fc->nav_mode = (uint8_t)NavMode();
  to_fc->target_position[0] = TargetPosition(N_WORLD_AXIS);
  to_fc->target_position[1] = TargetPosition(E_WORLD_AXIS);
  to_fc->target_position[2] = TargetPosition(D_WORLD_AXIS);
  to_fc->transit_speed = TransitSpeed();
  to_fc->target_heading = TargetHeading();
  to_fc->heading_rate = HeadingRate();

  if (to_fc == &to_fc_) UpdateCRCToFlightCtrl();
}

// -----------------------------------------------------------------------------
void UpdatePositionToFlightCtrl(enum Sensors sensor)
{
  float current_position[3];
  uint32_t status;

  if ((sensor == UBLOX) && (ActiveNavSensorBits() & SENSOR_BIT_UBLOX))
  {
    UBXToMeters(UBXGeodeticPositionVector(), current_position);
    current_position[D_WORLD_AXIS] = -PressureAltitude();
    status = UBXStatus();
  }
  else if ((sensor == VISION) && (ActiveNavSensorBits() & SENSOR_BIT_VISION))
  {
    current_position[N_WORLD_AXIS] = VisionPosition(N_WORLD_AXIS);
    current_position[E_WORLD_AXIS] = VisionPosition(E_WORLD_AXIS);
    current_position[D_WORLD_AXIS] = VisionPosition(D_WORLD_AXIS);
    status = VisionStatus();
  }
  else
  {
    return;
  }

  struct ToFlightCtrl * to_fc = &to_fc_;
  if (comms_ongoing_)
  {
    to_fc = &to_fc_buffer_;
    pending_update_bits_ |= PENDING_UPDATE_BIT_POSITION;
  }

  to_fc->position[N_WORLD_AXIS] = current_position[N_WORLD_AXIS];
  to_fc->position[E_WORLD_AXIS] = current_position[E_WORLD_AXIS];
  to_fc->position[D_WORLD_AXIS] = current_position[D_WORLD_AXIS];

  if (status)
    to_fc->status |= NAV_STATUS_BIT_POSITION_DATA_OK;
  else
    to_fc->status &= ~NAV_STATUS_BIT_POSITION_DATA_OK;

  if (to_fc == &to_fc_) UpdateCRCToFlightCtrl();
}

// -----------------------------------------------------------------------------
void UpdateVelocityToFlightCtrl(enum Sensors sensor)
{
  float velocity[3];
  uint32_t status;

  if ((sensor == UBLOX) && (ActiveNavSensorBits() & SENSOR_BIT_UBLOX))
  {
    velocity[N_WORLD_AXIS] = (float)UBXVelNED()->velocity_north * 1.0e-2;
    velocity[E_WORLD_AXIS] = (float)UBXVelNED()->velocity_east * 1.0e-2;
    velocity[D_WORLD_AXIS] = (float)UBXVelNED()->velocity_down * 1.0e-2;
    status = (UBXVelNED()->speed_accuracy < 100) && !UBXDataStale();
  }
  else if ((sensor == VISION) && (ActiveNavSensorBits() & SENSOR_BIT_VISION))
  {
    velocity[N_WORLD_AXIS] = KalmanVelocity(N_WORLD_AXIS);
    velocity[E_WORLD_AXIS] = KalmanVelocity(E_WORLD_AXIS);
    velocity[D_WORLD_AXIS] = KalmanVelocity(D_WORLD_AXIS);
    status = VisionStatus();
  }
  else
  {
    return;
  }

  struct ToFlightCtrl * to_fc = &to_fc_;
  if (comms_ongoing_)
  {
    to_fc = &to_fc_buffer_;
    pending_update_bits_ |= PENDING_UPDATE_BIT_VELOCITY;
  }

  to_fc->velocity[N_WORLD_AXIS] = velocity[N_WORLD_AXIS];
  to_fc->velocity[E_WORLD_AXIS] = velocity[E_WORLD_AXIS];
  to_fc->velocity[D_WORLD_AXIS] = velocity[D_WORLD_AXIS];

  if (status)
    to_fc->status |= NAV_STATUS_BIT_VELOCITY_DATA_OK;
  else
    to_fc->status &= ~NAV_STATUS_BIT_VELOCITY_DATA_OK;

  if (to_fc == &to_fc_) UpdateCRCToFlightCtrl();
}


// =============================================================================
// Private functions:

static void CopyPendingToFlightControlData(void)
{
  if (pending_update_bits_ & PENDING_UPDATE_BIT_HEADING_CORRECTION)
  {
    to_fc_.heading_correction_quat_0 = to_fc_buffer_.heading_correction_quat_0;
    to_fc_.heading_correction_quat_z = to_fc_buffer_.heading_correction_quat_z;
    to_fc_.status = (to_fc_.status & ~NAV_STATUS_BIT_HEADING_DATA_OK)
      | (to_fc_buffer_.status & NAV_STATUS_BIT_HEADING_DATA_OK);
  }

  if (pending_update_bits_ & PENDING_UPDATE_BIT_NAVIGATION)
  {
    to_fc_.nav_mode = to_fc_buffer_.nav_mode;
    to_fc_.target_position[0] = to_fc_buffer_.target_position[0];
    to_fc_.target_position[1] = to_fc_buffer_.target_position[1];
    to_fc_.target_position[2] = to_fc_buffer_.target_position[2];
    to_fc_.transit_speed = to_fc_buffer_.transit_speed;
    to_fc_.target_heading = to_fc_buffer_.target_heading;
    to_fc_.heading_rate = to_fc_buffer_.heading_rate;
    to_fc_.status = (to_fc_.status & ~NAV_STATUS_BIT_LOW_PRECISION_VERTICAL)
      | (to_fc_buffer_.status & NAV_STATUS_BIT_LOW_PRECISION_VERTICAL);
  }

  if (pending_update_bits_ & PENDING_UPDATE_BIT_POSITION)
  {
    to_fc_.position[N_WORLD_AXIS] = to_fc_buffer_.position[N_WORLD_AXIS];
    to_fc_.position[E_WORLD_AXIS] = to_fc_buffer_.position[E_WORLD_AXIS];
    to_fc_.position[D_WORLD_AXIS] = to_fc_buffer_.position[D_WORLD_AXIS];
    to_fc_.status = (to_fc_.status & ~NAV_STATUS_BIT_POSITION_DATA_OK)
      | (to_fc_buffer_.status & NAV_STATUS_BIT_POSITION_DATA_OK);
  }

  if (pending_update_bits_ & PENDING_UPDATE_BIT_VELOCITY)
  {
    to_fc_.velocity[N_WORLD_AXIS] = to_fc_buffer_.velocity[N_WORLD_AXIS];
    to_fc_.velocity[E_WORLD_AXIS] = to_fc_buffer_.velocity[E_WORLD_AXIS];
    to_fc_.velocity[D_WORLD_AXIS] = to_fc_buffer_.velocity[D_WORLD_AXIS];
    to_fc_.status = (to_fc_.status & ~NAV_STATUS_BIT_VELOCITY_DATA_OK)
      | (to_fc_buffer_.status & NAV_STATUS_BIT_VELOCITY_DATA_OK);
  }

  if (pending_update_bits_) UpdateCRCToFlightCtrl();
  pending_update_bits_ = 0;
}

// -----------------------------------------------------------------------------
static void UpdateCRCToFlightCtrl(void)
{
  to_fc_.crc = CRCCCITT((uint8_t *)&to_fc_, sizeof(struct ToFlightCtrl) - 2);
}