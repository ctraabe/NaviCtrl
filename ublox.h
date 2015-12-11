#ifndef UBLOX_H_
#define UBLOX_H_


#include <inttypes.h>


struct UBXPosLLH
{
  uint32_t gps_ms_time_of_week;
  int32_t longitude;
  int32_t latitude;
  int32_t height_above_ellipsoid;
  int32_t height_mean_sea_level;
  uint32_t horizontal_accuracy;
  uint32_t vertical_accuracy;
} __attribute__((packed));

struct UBXVelNED
{
  uint32_t gps_ms_time_of_week;
  int32_t velocity_north;
  int32_t velocity_east;
  int32_t velocity_down;
  uint32_t total_speed;
  uint32_t horizontal_speed;
  int32_t course;
  uint32_t speed_accuracy;
  uint32_t course_accuracy;
} __attribute__((packed));

struct UBXSol
{
  uint32_t gps_ms_time_of_week;
  int32_t fractional_time_of_week;
  int16_t gps_week;
  uint8_t gps_fix_type;
  uint8_t gps_fix_status_flags;
  int32_t ecef_x_coordinate;
  int32_t ecef_y_coordinate;
  int32_t ecef_z_coordinate;
  uint32_t coordinate_accuracy;
  int32_t ecef_x_velocity;
  int32_t ecef_y_velocity;
  int32_t ecef_z_velocity;
  uint32_t velocity_accuracy;
  uint16_t position_dop;
  uint8_t reserved1;
  uint8_t number_of_satelites_used;
  uint32_t reserved2;
} __attribute__((packed));

struct UBXTimeUTC
{
  uint32_t gps_ms_time_of_week;
  uint32_t t_acc;
  int32_t nano;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
} __attribute__((packed));


// =============================================================================
// Accessors:

const struct UBXPosLLH * UBXPosLLH(void);

// -----------------------------------------------------------------------------
const struct UBXVelNED * UBXVelNED(void);

// -----------------------------------------------------------------------------
const struct UBXSol * UBXSol(void);

// -----------------------------------------------------------------------------
const struct UBXTimeUTC * UBXTimeUTC(void);


// =============================================================================
// Public functions:

void UBloxInit(void);

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_buffer_) by the Rx interrupt handler. Each byte is passed to the
// appropriate Rx handler, which may place it into the temporary data buffer
// (data_buffer_).
void ProcessIncomingUBlox(void);

// -----------------------------------------------------------------------------
void UBloxUARTHandler(void);


#endif  // UBLOX_H_
