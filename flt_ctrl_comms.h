#ifndef FLT_CTRL_COMMS_H_
#define FLT_CTRL_COMMS_H_


#include <inttypes.h>
#include <stddef.h>


struct FromFltCtrl {
  int16_t accelerometer[3];
  int16_t gyro[3];
  float quaternion[4];
#ifdef LOG_FLT_CTRL_DEBUG_TO_SD
  int16_t sbus_pitch;
  int16_t sbus_roll;
  int16_t sbus_yaw;
  int16_t sbus_thrust;
  int16_t sbus_on_off;
  uint16_t state;
  uint16_t battery_voltage;
  float heading;
  float quaternion_command[4];
  float heading_command;
  float angular_command[3];
  float attitude_integral[3];
  float quaternion_model[4];
  uint16_t motor_setpoints[8];
  uint16_t timestamp;
#endif
} __attribute__((packed));


// =============================================================================
// Accessors:

const int16_t * AccelerometerVector(void);

// -----------------------------------------------------------------------------
const int16_t * GyroVector(void);

// -----------------------------------------------------------------------------
const float * Quat(void);

// -----------------------------------------------------------------------------
const struct FromFltCtrl * FromFltCtrl(void);

// -----------------------------------------------------------------------------
uint16_t FromFltCtrlCRC(void);


// =============================================================================
// Public functions:

void FltCtrlCommsInit(void);

// -----------------------------------------------------------------------------
// This function pulls the interrupt line down for about 1 microsecond.
void NotifyFltCtrl(void);

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_buffer_) by the Rx interrupt handler.
void ProcessIncomingFltCtrlByte(uint8_t byte);

// -----------------------------------------------------------------------------
void PrepareFltCtrlDataExchange(void);


#endif  // FLT_CTRL_COMMS_H_
