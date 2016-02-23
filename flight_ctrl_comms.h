#ifndef FLT_CTRL_COMMS_H_
#define FLT_CTRL_COMMS_H_


#include <inttypes.h>
#include <stddef.h>

#include "constants.h"
#include "navigation.h"


  struct FromFlightCtrl {
    uint16_t timestamp;
    uint8_t nav_mode_request;
    uint8_t state;
    float accelerometer[3];
    float gyro[3];
    float quaternion[4];
#ifdef LOG_FLT_CTRL_DEBUG_TO_SD
    int16_t sbus_pitch;
    int16_t sbus_roll;
    int16_t sbus_yaw;
    int16_t sbus_thrust;
    uint16_t battery_voltage;
    float pressure_altitude;
    float thrust_command;
    float heading_command;
    float angular_command[3];
    float kalman_p_dot;
    float kalman_q_dot;
    float vertical_speed;
    float vertical_acceleration;
#endif
} __attribute__((packed));

enum FlightCtrlStateBits {
  FC_STATE_BIT_MOTORS_INHIBITED      = 1<<0,
  FC_STATE_BIT_INITIALIZED           = 1<<1,
  FC_STATE_BIT_STARTING              = 1<<2,
  FC_STATE_BIT_MOTORS_RUNNING        = 1<<3,
};


// =============================================================================
// Accessors:

float Accelerometer(enum BodyAxes axis);

// -----------------------------------------------------------------------------
const volatile float * AccelerometerVector(void);

// -----------------------------------------------------------------------------
enum FlightCtrlStateBits FlightCtrlState(void);

// -----------------------------------------------------------------------------
uint16_t FlightCtrlTimestamp(void);

// -----------------------------------------------------------------------------
float Gyro(enum BodyAxes axis);

// -----------------------------------------------------------------------------
const volatile float * GyroVector(void);

// -----------------------------------------------------------------------------
const volatile float * Quat(void);

// -----------------------------------------------------------------------------
enum NavMode RequestedNavMode(void);

// -----------------------------------------------------------------------------
uint32_t RequestedNavRoute(void);

// -----------------------------------------------------------------------------
const volatile struct FromFlightCtrl * FromFlightCtrl(void);

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
