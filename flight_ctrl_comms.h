#ifndef FLT_CTRL_COMMS_H_
#define FLT_CTRL_COMMS_H_


#include <inttypes.h>
#include <stddef.h>

#include "constants.h"
#include "navigation.h"
#include "sensor_enumeration.h"


struct FromFlightCtrl {
  uint16_t timestamp;
  uint8_t nav_mode_request;
  uint8_t state;
  float accelerometer[3];
  float gyro[3];
  float quaternion[4];
  float pressure_altitude;
} __attribute__((packed));

struct ToFlightCtrl {
  uint16_t version;
  uint8_t nav_mode;
  uint8_t status;
  float position[3];
  float velocity[3];
  float heading_correction_quat_0;
  float heading_correction_quat_z;
  float target_position[3];
  float transit_speed;
  float target_heading;
  float heading_rate;
  uint16_t crc;
} __attribute__((packed));

enum FlightCtrlStateBits {
  FC_STATE_BIT_MOTORS_INHIBITED      = 1<<0,
  FC_STATE_BIT_INITIALIZED           = 1<<1,
  FC_STATE_BIT_STARTING              = 1<<2,
  FC_STATE_BIT_MOTORS_RUNNING        = 1<<3,
  FC_STATE_BIT_INITIALIZATION_TOGGLE = 1<<4,
  FC_STATE_BIT_LOST_CONTROL_LINK     = 1<<5,
};

enum NavStatusBits {
  NAV_STATUS_BIT_HEADING_DATA_OK         = 1<<0,
  NAV_STATUS_BIT_POSITION_DATA_OK        = 1<<1,
  NAV_STATUS_BIT_VELOCITY_DATA_OK        = 1<<2,
  NAV_STATUS_BIT_LOW_PRECISION_VERTICAL  = 1<<3,
};


// =============================================================================
// Accessors:

float Accelerometer(enum BodyAxes axis);

// -----------------------------------------------------------------------------
const float * AccelerometerVector(void);

// -----------------------------------------------------------------------------
uint32_t FlightCtrlCommsOngoing(void);

// -----------------------------------------------------------------------------
uint8_t FlightCtrlState(void);

// -----------------------------------------------------------------------------
uint16_t FlightCtrlTimestamp(void);

// -----------------------------------------------------------------------------
float Gyro(enum BodyAxes axis);

// -----------------------------------------------------------------------------
const float * GyroVector(void);

// -----------------------------------------------------------------------------
uint32_t NewDataFromFlightCtrl(void);

// -----------------------------------------------------------------------------
float PressureAltitude(void);

// -----------------------------------------------------------------------------
const float * PositionVector(void);

// -----------------------------------------------------------------------------
const float * Quat(void);

// -----------------------------------------------------------------------------
uint32_t RCSwitch(void);

// -----------------------------------------------------------------------------
enum NavMode RequestedNavMode(void);

// -----------------------------------------------------------------------------
uint32_t RequestedNavRoute(void);

// -----------------------------------------------------------------------------
const struct FromFlightCtrl * FromFlightCtrl(void);

// -----------------------------------------------------------------------------
uint16_t FromFlightCtrlCRC(void);


// =============================================================================
// Public functions:

void ClearNewDataFromFlightCtrlFlag(void);

// -----------------------------------------------------------------------------
void FlightCtrlCommsInit(void);

// -----------------------------------------------------------------------------
// This function pulls the interrupt line down for about 1 microsecond.
void NotifyFlightCtrl(void);

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_buffer_) by the Rx interrupt handler.
void ProcessIncomingFlightCtrlByte(uint8_t byte);

// -----------------------------------------------------------------------------
void PrepareFlightCtrlDataExchange(void);

// -----------------------------------------------------------------------------
void SetFlightCtrlCommsOngoingFlag(void);

// -----------------------------------------------------------------------------
void UpdateHeadingCorrectionToFlightCtrl(enum Sensors sensor);

// -----------------------------------------------------------------------------
void UpdateNavigationToFlightCtrl(void);

// -----------------------------------------------------------------------------
void UpdatePositionToFlightCtrl(enum Sensors sensor);

// -----------------------------------------------------------------------------
void UpdateVelocityToFlightCtrl(enum Sensors sensor);


#endif  // FLT_CTRL_COMMS_H_
