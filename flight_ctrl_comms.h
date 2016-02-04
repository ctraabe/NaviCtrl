#ifndef FLT_CTRL_COMMS_H_
#define FLT_CTRL_COMMS_H_


#include <inttypes.h>
#include <stddef.h>


struct FromFlightCtrl {
  uint16_t timestamp;
  uint16_t state;
  int16_t accelerometer[3];
  int16_t gyro[3];
  float quaternion[4];
#ifdef LOG_FLT_CTRL_DEBUG_TO_SD
  int16_t sbus_pitch;
  int16_t sbus_roll;
  int16_t sbus_yaw;
  int16_t sbus_thrust;
  int16_t sbus_on_off;
  uint16_t battery_voltage;
  float heading;
  float quaternion_command[4];
  float heading_command;
  float angular_command[3];
  float attitude_integral[3];
  float quaternion_model[4];
  uint16_t motor_setpoints[8];
#endif
} __attribute__((packed));

enum FlightCtrlStateBits {
  FC_STATE_BIT_MOTORS_INHIBITED   = 1<<0,
  FC_STATE_BIT_INITIALIZED        = 1<<1,
  FC_STATE_BIT_STARTING           = 1<<2,
  FC_STATE_BIT_MOTORS_RUNNING     = 1<<3,
  FC_STATE_BIT_IN_AIR             = 1<<4,
  FC_STATE_BIT_LOW_BATTERY        = 1<<5,
  FC_STATE_BIT_EMERGENCY_LANDING  = 1<<6,
};


// =============================================================================
// Accessors:

const int16_t * AccelerometerVector(void);

// -----------------------------------------------------------------------------
enum FlightCtrlStateBits FlightCtrlState(void);

// -----------------------------------------------------------------------------
uint16_t FlightCtrlTimestamp(void);

// -----------------------------------------------------------------------------
const int16_t * GyroVector(void);

// -----------------------------------------------------------------------------
const float * Quat(void);

// -----------------------------------------------------------------------------
const struct FromFlightCtrl * FromFlightCtrl(void);

// -----------------------------------------------------------------------------
uint16_t FromFlightCtrlCRC(void);


// =============================================================================
// Public functions:

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


#endif  // FLT_CTRL_COMMS_H_
