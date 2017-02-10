#include "navigation.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "attitude.h"
#include "custom_math.h"
#include "ff.h"  // from libfatfs
#include "flight_ctrl_comms.h"
#include "sd_card.h"
#include "timing.h"
#include "uart1.h"
#include "ublox.h"
#include "union_types.h"
#include "vector.h"
#include "vision.h"


// =============================================================================
// Private data:

#define N_ROUTES (3)
#define MAX_WAYPOINTS (32)
// TODO: make this a set-able parameter
#define DEFAULT_TRANSIT_SPEED (1)  // m/s

union S32Float {
  int32_t s32;
  float f;
};

struct Waypoint {
  uint16_t type;
  uint16_t wait_ms;
  union S32Float target_position[3];
  float transit_speed;
  float radius;
  float target_heading;
  float heading_rate;
  float heading_range;
} __attribute__((packed));

enum RouteNumber {
  ROUTE_1 = 0,
  ROUTE_2 = 1,
  ROUTE_3 = 2,
};

static uint32_t n_waypoints_[N_ROUTES] = { 0, 0, 0 };
static struct Waypoint waypoints_[N_ROUTES][MAX_WAYPOINTS] = { 0 };
static enum NavMode mode_ = NAV_MODE_OFF;
static enum NavError nav_error_ = NAV_ERROR_NONE;
static enum SensorBits active_nav_sensors_ = SENSOR_BIT_UBLOX
  | SENSOR_BIT_LSM303DL;

static float delta_postion_[3] = { 0.0 },
  target_position_[3] = { 0.0 };
static float current_heading_ = 0.0, delta_heading_ = 0.0,
  target_heading_ = 0.0;

static const struct Waypoint * current_waypoint_ = &waypoints_[ROUTE_1][0];
static const struct Waypoint * final_waypoint_ = &waypoints_[ROUTE_1][0];

static int32_t geodetic_home_[3];  // (deg * 1e7, deg * 1e7, mm)
static float ubx_longitude_to_meters_;


// =============================================================================
// Private function declarations:

static uint32_t NextWaypointValid(void);
static void SetActiveNavSensors(void);
static void SetTargetPosition(const struct Waypoint * const waypoint);


// =============================================================================
// Accessors:

enum SensorBits ActiveNavSensorBits(void)
{
  return active_nav_sensors_;
}

// -----------------------------------------------------------------------------
float CurrentHeading(void)
{
  return current_heading_;
}

// -----------------------------------------------------------------------------
int32_t GeodeticHome(enum GeoAxes axis)
{
  return geodetic_home_[axis];
}

// -----------------------------------------------------------------------------
float HeadingRate(void)
{
  return current_waypoint_->heading_rate;
}

// -----------------------------------------------------------------------------
float NavDeltaPosition(enum WorldAxes axis)
{
  return delta_postion_[axis];
}

// -----------------------------------------------------------------------------
enum NavMode NavMode(void)
{
  return mode_;
}

// -----------------------------------------------------------------------------
float TargetHeading(void)
{
  return target_heading_;
}

// -----------------------------------------------------------------------------
float TargetPosition(enum WorldAxes axis)
{
  return target_position_[axis];
}

// -----------------------------------------------------------------------------
float TransitSpeed(void)
{
  if (mode_ == NAV_MODE_AUTO) return current_waypoint_->transit_speed;
  return DEFAULT_TRANSIT_SPEED;
}

// -----------------------------------------------------------------------------
float UBXLongitudeToMeters(void)
{
  return ubx_longitude_to_meters_;
}


// =============================================================================
// Public functions:

// Try to load waypoints from WP_LIST.CSV on the SD card (if inserted).
void NavigationInit(void)
{
#if defined(OBSTACLE_AVOIDANCE_A)
  n_waypoints_[ROUTE_1] = 2;

  waypoints_[ROUTE_1][0].type = WP_TYPE_VISION_RELATIVE;
  waypoints_[ROUTE_1][0].target_position[0].f = 0.0;
  waypoints_[ROUTE_1][0].target_position[1].f = 0.0;
  waypoints_[ROUTE_1][0].target_position[2].f = -1.0;
  waypoints_[ROUTE_1][0].transit_speed = 0.5;
  waypoints_[ROUTE_1][0].radius = 1.0;
  waypoints_[ROUTE_1][0].target_heading = 0.0;
  waypoints_[ROUTE_1][0].heading_rate = 0.1;
  waypoints_[ROUTE_1][0].heading_range = 0.1;
  waypoints_[ROUTE_1][0].wait_ms = 0;

  waypoints_[ROUTE_1][1].type = WP_TYPE_VISION_RELATIVE;
  waypoints_[ROUTE_1][1].target_position[0].f = 0.0;
  waypoints_[ROUTE_1][1].target_position[1].f = -1.5;
  waypoints_[ROUTE_1][1].target_position[2].f = -1.0;
  waypoints_[ROUTE_1][1].transit_speed = 0.5;
  waypoints_[ROUTE_1][1].radius = 1.0;
  waypoints_[ROUTE_1][1].target_heading = 0.0;
  waypoints_[ROUTE_1][1].heading_rate = 0.1;
  waypoints_[ROUTE_1][1].heading_range = 0.1;
  waypoints_[ROUTE_1][1].wait_ms = 0;
#elif defined(OBSTACLE_AVOIDANCE_B)
  n_waypoints_[ROUTE_1] = 3;

  waypoints_[ROUTE_1][0].type = WP_TYPE_VISION_RELATIVE;
  waypoints_[ROUTE_1][0].target_position[0].f = 2.0;
  waypoints_[ROUTE_1][0].target_position[1].f = 0.0;
  waypoints_[ROUTE_1][0].target_position[2].f = -0.8;
  waypoints_[ROUTE_1][0].transit_speed = 0.5;
  waypoints_[ROUTE_1][0].radius = 1.0;
  waypoints_[ROUTE_1][0].target_heading = 0.0;
  waypoints_[ROUTE_1][0].heading_rate = 0.1;
  waypoints_[ROUTE_1][0].heading_range = 0.1;
  waypoints_[ROUTE_1][0].wait_ms = 0;

  waypoints_[ROUTE_1][1].type = WP_TYPE_VISION_RELATIVE;
  waypoints_[ROUTE_1][1].target_position[0].f = 2.0;
  waypoints_[ROUTE_1][1].target_position[1].f = -1.0;
  waypoints_[ROUTE_1][1].target_position[2].f = -0.8;
  waypoints_[ROUTE_1][1].transit_speed = 0.5;
  waypoints_[ROUTE_1][1].radius = 1.0;
  waypoints_[ROUTE_1][1].target_heading = 0.0;
  waypoints_[ROUTE_1][1].heading_rate = 0.1;
  waypoints_[ROUTE_1][1].heading_range = 0.1;
  waypoints_[ROUTE_1][1].wait_ms = 0;

  waypoints_[ROUTE_1][2].type = WP_TYPE_VISION_RELATIVE;
  waypoints_[ROUTE_1][2].target_position[0].f = 2.0;
  waypoints_[ROUTE_1][2].target_position[1].f = -1.0;
  waypoints_[ROUTE_1][2].target_position[2].f = -0.8;
  waypoints_[ROUTE_1][2].transit_speed = 0.5;
  waypoints_[ROUTE_1][2].radius = 1.0;
  waypoints_[ROUTE_1][2].target_heading = 0.0;
  waypoints_[ROUTE_1][2].heading_rate = 0.1;
  waypoints_[ROUTE_1][2].heading_range = 0.1;
  waypoints_[ROUTE_1][2].wait_ms = 0;

  n_waypoints_[ROUTE_2] = 5;

  waypoints_[ROUTE_2][0].type = WP_TYPE_VISION_RELATIVE;
  waypoints_[ROUTE_2][0].target_position[0].f = 2.0;
  waypoints_[ROUTE_2][0].target_position[1].f = 0.0;
  waypoints_[ROUTE_2][0].target_position[2].f = -0.8;
  waypoints_[ROUTE_2][0].transit_speed = 0.5;
  waypoints_[ROUTE_2][0].radius = 1.0;
  waypoints_[ROUTE_2][0].target_heading = 0.0;
  waypoints_[ROUTE_2][0].heading_rate = 0.1;
  waypoints_[ROUTE_2][0].heading_range = 0.1;
  waypoints_[ROUTE_2][0].wait_ms = 0;

  waypoints_[ROUTE_2][1].type = WP_TYPE_VISION_RELATIVE;
  waypoints_[ROUTE_2][1].target_position[0].f = 2.0;
  waypoints_[ROUTE_2][1].target_position[1].f = -1.0;
  waypoints_[ROUTE_2][1].target_position[2].f = -0.8;
  waypoints_[ROUTE_2][1].transit_speed = 0.5;
  waypoints_[ROUTE_2][1].radius = 1.0;
  waypoints_[ROUTE_2][1].target_heading = 0.0;
  waypoints_[ROUTE_2][1].heading_rate = 0.1;
  waypoints_[ROUTE_2][1].heading_range = 0.1;
  waypoints_[ROUTE_2][1].wait_ms = 0;

  waypoints_[ROUTE_2][2].type = WP_TYPE_VISION_RELATIVE;
  waypoints_[ROUTE_2][2].target_position[0].f = 2.0;
  waypoints_[ROUTE_2][2].target_position[1].f = -1.0;
  waypoints_[ROUTE_2][2].target_position[2].f = -0.8;
  waypoints_[ROUTE_2][2].transit_speed = 0.5;
  waypoints_[ROUTE_2][2].radius = 1.0;
  waypoints_[ROUTE_2][2].target_heading = 0.0;
  waypoints_[ROUTE_2][2].heading_rate = 0.1;
  waypoints_[ROUTE_2][2].heading_range = 0.1;
  waypoints_[ROUTE_2][2].wait_ms = 0;

  waypoints_[ROUTE_2][3].type = WP_TYPE_VISION_RELATIVE;
  waypoints_[ROUTE_2][3].target_position[0].f = 2.0;
  waypoints_[ROUTE_2][3].target_position[1].f = -1.0;
  waypoints_[ROUTE_2][3].target_position[2].f = -0.8;
  waypoints_[ROUTE_2][3].transit_speed = 0.5;
  waypoints_[ROUTE_2][3].radius = 1.0;
  waypoints_[ROUTE_2][3].target_heading = 0.0;
  waypoints_[ROUTE_2][3].heading_rate = 0.1;
  waypoints_[ROUTE_2][3].heading_range = 0.1;
  waypoints_[ROUTE_2][3].wait_ms = 0;

  waypoints_[ROUTE_2][4].type = WP_TYPE_VISION_RELATIVE;
  waypoints_[ROUTE_2][4].target_position[0].f = 2.0;
  waypoints_[ROUTE_2][4].target_position[1].f = -1.0;
  waypoints_[ROUTE_2][4].target_position[2].f = -0.8;
  waypoints_[ROUTE_2][4].transit_speed = 0.5;
  waypoints_[ROUTE_2][4].radius = 1.0;
  waypoints_[ROUTE_2][4].target_heading = 0.0;
  waypoints_[ROUTE_2][4].heading_rate = 0.1;
  waypoints_[ROUTE_2][4].heading_range = 0.1;
  waypoints_[ROUTE_2][4].wait_ms = 0;
#else
  if (!SDCardFSMounted()) return;

  FIL file;
  char filename[12] = "WP_LIST.CSV", line[256];
  if (f_open(&file, filename, FA_READ) != FR_OK)
  {
    UART1Printf("Failed to open %s", filename);
    return;
  }

  UART1Printf("Opened file %s", filename);

  // Clear the waypoints.
  for (size_t i = N_ROUTES; i--; ) n_waypoints_[i] = 0;

  // Read WP_LIST.CSV line by line.
  uint32_t line_number = 0, route_index = 0;
  struct Waypoint * waypoint_ptr = &waypoints_[0][0];
  while (f_gets(line, sizeof(line), &file))
  {
    line_number++;

    // Check for a comment line.
    if (line[0] == '/') continue;  // Skip to the next line

    // Check for route number.
    if (line[0] == 'R')
    {
      route_index = atoi(&line[6]) - 1;
      if (route_index < N_ROUTES)
      {
        // Clear the route again (for safety).
        n_waypoints_[route_index] = 0;
        waypoint_ptr = &waypoints_[route_index][0];
      }
      else
      {
        nav_error_ = NAV_ERROR_SD_ROUTE_NUMBER;
        UART1Printf("navigation: invalid route number in WP_LIST.CSV line %i",
          line_number);
        goto CLOSE_AND_RETURN;
      }
      continue;  // Skip to the next line
    }

    // Loop through the comma separated data in the line.
    char * line_ptr = line;
    uint8_t * data_ptr = (void *)waypoint_ptr;
    uint32_t data_index = 0;
    for (;;)
    {
      if ((*line_ptr > 0x1F) && (*line_ptr != ','))  // Data
      {
        // Check that number of waypoints does not exceed the maximum.
        if (n_waypoints_[route_index] == MAX_WAYPOINTS)
        {
          nav_error_ = NAV_ERROR_SD_WAYPOINT_COUNT;
          UART1Printf("navigation: too many waypoints in WP_LIST.CSV line %i",
            line_number);
          goto CLOSE_AND_RETURN;
        }

        // The first 2 entries of the Waypoint structure are uint16, followed by
        // 2 int32s or floats (depending on WP type), then 6 floats.
        if (data_index < 2)
        {
          *(uint16_t *)data_ptr = (uint16_t)atoi(line_ptr);
          data_ptr += sizeof(uint16_t);
        }
        else if (data_index < 4)
        {
          if (waypoint_ptr->type & WP_TYPE_BIT_GEODETIC)
            *(int32_t *)data_ptr = atoi(line_ptr);
          else
            *(float *)data_ptr = atof(line_ptr);
          data_ptr += sizeof(uint32_t);
        }
        else if (data_index < 10)
        {
          *(float *)data_ptr = atof(line_ptr);
          data_ptr += sizeof(float);
        }
        data_index++;
        // Move the pointer past the next comma (if possible).
        do { line_ptr++; } while ((*line_ptr > 0x1F) && (*line_ptr != ','));
      }

      if (*line_ptr == 0)  // End of line
      {
        // Ignore blank lines
        if (data_index == 0) continue;  // Skip to the next line

        // Check that the correct number of data entries were read.
        if (data_index == 10)
        {
          // Post processing:
          // Convert degrees to radians.
          waypoint_ptr->target_heading *= M_PI / 180.0;
          waypoint_ptr->heading_rate *= M_PI / 180.0;
          waypoint_ptr->heading_range *= M_PI / 180.0;

          // Change altitude to positive down.
          waypoint_ptr->target_position[2].f
            = -waypoint_ptr->target_position[2].f;

          // Safety checks.
          waypoint_ptr->transit_speed = fabs(waypoint_ptr->transit_speed);
          waypoint_ptr->radius = fabs(waypoint_ptr->radius);
          waypoint_ptr->heading_rate = fabs(waypoint_ptr->heading_rate);
          waypoint_ptr->heading_range = fabs(waypoint_ptr->heading_range);

          // Advance to the next waypoint memory location.
          waypoint_ptr++;
          n_waypoints_[route_index]++;
        }
        else
        {
          nav_error_ = NAV_ERROR_SD_DATA_ROWS;
          UART1Printf("navigation: incompatible number of entries in"
            " WP_LIST.CSV line %i", line_number);
          goto CLOSE_AND_RETURN;
        }
        break;  // Exit the while loop and proceed to the next line
      }

      line_ptr++;  // Next character
    }
  }
#endif

  // Report the loaded waypoints.
  for (uint32_t i = 0; i < 3; i++)
  {
    UART1Printf("Route %i:", i + 1);
    for (uint32_t j = 0; j < n_waypoints_[i]; j++)
    {
      float position[2];
      if (waypoints_[i][j].type & WP_TYPE_BIT_GEODETIC)
      {
        position[0] = waypoints_[i][j].target_position[0].s32 * 1.0e-7;
        position[1] = waypoints_[i][j].target_position[1].s32 * 1.0e-7;
      }
      else
      {
        position[0] = waypoints_[i][j].target_position[0].f;
        position[1] = waypoints_[i][j].target_position[1].f;
      }
      UART1Printf("%02i: (%+06.2f %+06.2f %+06.2f) %4.2f %4.2f %+04.0f %03.0f"
        " %03.0f %i %i", j + 1,
        position[0],
        position[1],
        -waypoints_[i][j].target_position[2].f,
        waypoints_[i][j].transit_speed,
        waypoints_[i][j].radius,
        waypoints_[i][j].target_heading * 180.0 / M_PI,
        waypoints_[i][j].heading_rate * 180.0 / M_PI,
        waypoints_[i][j].heading_range * 180.0 / M_PI,
        waypoints_[i][j].wait_ms,
        waypoints_[i][j].type);
    }
  }

#if defined(OBSTACLE_AVOIDANCE_A) || defined(OBSTACLE_AVOIDANCE_B)
#else
  UART1Printf("End of %s", filename);

  CLOSE_AND_RETURN:
  f_close(&file);
#endif
}

// -----------------------------------------------------------------------------
void UpdateNavigation(void)
{
#ifndef OBSTACLE_AVOIDANCE_A
  static float radius_squared = 1.0;
  static uint32_t next_waypoint_time = 0, waypoint_reached = 0;
#endif

  static uint32_t state_pv = 0;
  if ((FlightCtrlState() ^ state_pv) & FC_STATE_BIT_INITIALIZATION_TOGGLE)
    SetGeodeticHome();
  state_pv = FlightCtrlState();
  current_heading_ = HeadingFromQuaternion((float *)Quat());

  if (RequestedNavMode() != mode_)
  {
    if (RequestedNavMode() == NAV_MODE_AUTO)
    {
#if defined(OBSTACLE_AVOIDANCE_A) || defined(OBSTACLE_AVOIDANCE_B)
      uint32_t route = ROUTE_1;
#else
      uint32_t route = RequestedNavRoute();
#endif
      if ((nav_error_ == NAV_ERROR_NONE) && n_waypoints_[route] > 0)
      {
        final_waypoint_ = &waypoints_[route][n_waypoints_[route]-1];
        // Allow continuation from NAV_MODE_HOLD.
        if ((current_waypoint_ >= final_waypoint_) || (current_waypoint_ <
          &waypoints_[route][0]))
        {
          current_waypoint_ = &waypoints_[route][0];
        }
        SetTargetPosition(current_waypoint_);
        target_heading_ = current_waypoint_->target_heading;
#ifndef OBSTACLE_AVOIDANCE_A
        radius_squared = current_waypoint_->radius * current_waypoint_->radius;
#endif
        SetActiveNavSensors();
        mode_ = NAV_MODE_AUTO;
      }
      else
      {
        mode_ = NAV_MODE_HOLD;
      }
    }
    else
    {
      mode_ = RequestedNavMode();
    }

    if (mode_ == NAV_MODE_HOLD)
    {
      Vector3Copy(PositionVector(), target_position_);
      target_heading_ = current_heading_;
    }
  }

  Vector3Subtract(PositionVector(), target_position_, delta_postion_);
  delta_heading_ = WrapToPlusMinusPi(current_heading_ - target_heading_);

  if (mode_ == NAV_MODE_HOLD)
  {
    SetActiveNavSensors();
  }

  if (mode_ == NAV_MODE_AUTO)
  {
#ifndef OBSTACLE_AVOIDANCE_A
    if (!waypoint_reached
      && (Vector3NormSquared(delta_postion_) < radius_squared)
      && (fabs(delta_heading_) < current_waypoint_->heading_range))
    {
      waypoint_reached = 1;
      next_waypoint_time = GetTimestampMillisFromNow(current_waypoint_->wait_ms)
        - 1;
    }
    if (waypoint_reached && TimestampInPast(next_waypoint_time)
      && NextWaypointValid())
    {
      waypoint_reached = 0;
      current_waypoint_++;
      SetTargetPosition(current_waypoint_);
      target_heading_ = current_waypoint_->target_heading;
      radius_squared = current_waypoint_->radius * current_waypoint_->radius;
      SetActiveNavSensors();
    }
  #ifdef OBSTACLE_AVOIDANCE_B
    if ((current_waypoint_ == &waypoints_[ROUTE_1][1])
      && VisionObstacleLocationVector()[0] != 0.0)
    {
      current_waypoint_ = &waypoints_[ROUTE_2][1];
      final_waypoint_ = &waypoints_[ROUTE_2][n_waypoints_[ROUTE_2]-1];
    }
  #endif
#else
    if (VisionObstacleLocationVector()[0] != 0.0)
      current_waypoint_ = &waypoints_[ROUTE_1][1];
    else
      current_waypoint_ = &waypoints_[ROUTE_1][0];
    SetTargetPosition(current_waypoint_);
    target_heading_ = current_waypoint_->target_heading;
#endif
  }

  UpdateNavigationToFlightCtrl();
}

// -----------------------------------------------------------------------------
void SetGeodeticHome(void)
{
  memcpy(geodetic_home_, &UBXPosLLH()->longitude, sizeof(geodetic_home_));

  ubx_longitude_to_meters_ = UBX_LATITUDE_TO_METERS
    * cos((float)geodetic_home_[LATITUDE] * 1.0e-7 * M_PI / 180.0);
}


// =============================================================================
// Private functions:

static uint32_t NextWaypointValid(void)
{
  if (current_waypoint_ == final_waypoint_) return 0;

  if (current_waypoint_[1].type & WP_TYPE_BIT_VISION) return VisionStatus();

  return UBXStatus();
}

// -----------------------------------------------------------------------------
static void SetActiveNavSensors(void)
{
  if ((mode_ == NAV_MODE_AUTO && current_waypoint_->type & WP_TYPE_BIT_VISION)
    || (mode_ != NAV_MODE_AUTO && VisionStatus()))
  {
    active_nav_sensors_ = SENSOR_BIT_VISION;
  }
  else
  {
    active_nav_sensors_ = SENSOR_BIT_UBLOX | SENSOR_BIT_LSM303DL;
  }
}

// -----------------------------------------------------------------------------
static void SetTargetPosition(const struct Waypoint * const waypoint)
{
  if (waypoint->type & WP_TYPE_BIT_GEODETIC)
  {
    target_position_[N_WORLD_AXIS] =
      (float)(waypoint->target_position[LATITUDE].s32
      - geodetic_home_[LATITUDE]) * UBX_LATITUDE_TO_METERS;
    target_position_[E_WORLD_AXIS] =
      (float)(waypoint->target_position[LONGITUDE].s32
      - geodetic_home_[LONGITUDE]) * ubx_longitude_to_meters_;
  }
  else
  {
    target_position_[N_WORLD_AXIS] = waypoint->target_position[N_WORLD_AXIS].f;
    target_position_[E_WORLD_AXIS] = waypoint->target_position[E_WORLD_AXIS].f;
  }
  target_position_[D_WORLD_AXIS] = waypoint->target_position[D_WORLD_AXIS].f;
}
