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
#define N_GO_HOME_WAYPOINTS (3)
#define GO_HOME_ALTITUDE (2.0) // m
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
static struct Waypoint go_home_[N_GO_HOME_WAYPOINTS] = { 0 };
static enum NavMode mode_ = NAV_MODE_OFF;
static enum NavError nav_error_ = NAV_ERROR_NONE;
static enum SensorBits active_nav_sensors_ = SENSOR_BIT_VISION;
static size_t route_ = ROUTE_1;

static float delta_postion_[3] = { 0.0 },
  target_position_[3] = { 0.0 };
static float delta_heading_ = 0.0, target_heading_ = 0.0;

static const struct Waypoint * current_waypoint_ = &waypoints_[ROUTE_1][0];
static const struct Waypoint * final_waypoint_ = &waypoints_[ROUTE_1][0];

static int32_t geodetic_home_[3];  // (deg * 1e7, deg * 1e7, mm)
static float ubx_longitude_to_meters_;


// =============================================================================
// Private function declarations:

static uint32_t NextWaypointValid(void);
static void SetTargetPosition(const struct Waypoint * const waypoint);


// =============================================================================
// Accessors:

enum SensorBits ActiveNavSensorBits(void)
{
  return active_nav_sensors_;
}

// -----------------------------------------------------------------------------
uint32_t CurrentWaypoint(void)
{
  return current_waypoint_ - &waypoints_[route_][0];
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
uint32_t Route(void)
{
  return route_;
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

void NavigationInit(void)
{
#ifdef HARDCODE_GEODETIC_HOME
  // Ina-shi, Nagano
  geodetic_home_[LONGITUDE] = 1380821430;
  geodetic_home_[LATITUDE] = 358082810;
  geodetic_home_[HEIGHT] = 0;
  ubx_longitude_to_meters_ = UBX_LATITUDE_TO_METERS
  * cos((float)geodetic_home_[LATITUDE] * 1.0e-7 * M_PI / 180.0);
#endif

  // Initialize "go home" waypoint speeds. etc.

  // First waypoint is directly up to "go home" altitude.
  go_home_[0].type = WP_TYPE_GPS_RELATIVE;
  go_home_[0].target_position[D_WORLD_AXIS].f = -GO_HOME_ALTITUDE;
  go_home_[0].transit_speed = 0.5;
  go_home_[0].radius = 1.0;
  go_home_[0].target_heading = 0.0;
  go_home_[0].heading_rate = 15.0 * M_PI / 180.0;
  go_home_[0].heading_range = 15.0 * M_PI / 180.0;
  go_home_[0].wait_ms = 0;

  // Second waypoint directly across to home.
  go_home_[1].type = WP_TYPE_GPS_RELATIVE;
  go_home_[1].target_position[N_WORLD_AXIS].f = 0.0;
  go_home_[1].target_position[E_WORLD_AXIS].f = 0.0;
  go_home_[1].target_position[D_WORLD_AXIS].f = -GO_HOME_ALTITUDE;
  go_home_[1].transit_speed = 1.0;
  go_home_[1].radius = 1.0;
  go_home_[1].target_heading = 0.0;
  go_home_[1].heading_rate = 15.0 * M_PI / 180.0;
  go_home_[1].heading_range = 15.0 * M_PI / 180.0;
  go_home_[1].wait_ms = 0;

  // Third waypoint descends vertically to the ground.
  go_home_[2].type = WP_TYPE_GPS_RELATIVE;
  go_home_[2].target_position[N_WORLD_AXIS].f = 0.0;
  go_home_[2].target_position[E_WORLD_AXIS].f = 0.0;
  go_home_[2].target_position[D_WORLD_AXIS].f = 999.0;
  go_home_[2].transit_speed = 0.5;
  go_home_[2].radius = 1.0;
  go_home_[2].target_heading = 0.0;
  go_home_[2].heading_rate = 15.0 * M_PI / 180.0;
  go_home_[2].heading_range = 15.0 * M_PI / 180.0;
  go_home_[2].wait_ms = 0;

  // Try to load waypoints from WP_LIST.CSV on the SD card (if inserted).
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

          // Swap the order of lat and lon to match UBX.
          if (waypoint_ptr->type & WP_TYPE_BIT_GEODETIC)
          {
            int32_t temp = waypoint_ptr->target_position[0].s32;
            waypoint_ptr->target_position[0].s32
              = waypoint_ptr->target_position[1].s32;
            waypoint_ptr->target_position[1].s32 = temp;
          }

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

  // Report the loaded waypoints.
  for (uint32_t i = 0; i < 3; i++)
  {
    UART1Printf("Route %i:", i + 1);
    for (uint32_t j = 0; j < n_waypoints_[i]; j++)
    {
      float position[2];
      if (waypoints_[i][j].type & WP_TYPE_BIT_GEODETIC)
      {
        position[0] = waypoints_[i][j].target_position[LATITUDE].s32 * 1.0e-7;
        position[1] = waypoints_[i][j].target_position[LONGITUDE].s32 * 1.0e-7;
      }
      else
      {
        position[0] = waypoints_[i][j].target_position[N_WORLD_AXIS].f;
        position[1] = waypoints_[i][j].target_position[E_WORLD_AXIS].f;
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

  UART1Printf("End of %s", filename);

  CLOSE_AND_RETURN:
  f_close(&file);
}

// -----------------------------------------------------------------------------
void UpdateNavigation(void)
{
  static float radius_squared = 1.0;
  static uint32_t next_waypoint_time = 0, waypoint_reached = 0;

#ifndef HARDCODE_GEODETIC_HOME
  static uint32_t state_pv = 0;
  if ((FlightCtrlState() ^ state_pv) & FC_STATE_BIT_INITIALIZATION_TOGGLE)
    SetGeodeticHome();
  state_pv = FlightCtrlState();
#endif

  // Mode switching logic.
  if (RequestedNavMode() != mode_)
  {
    switch (RequestedNavMode())
    {
      case NAV_MODE_AUTO:
      {
        route_ = RequestedNavRoute();
        if ((nav_error_ == NAV_ERROR_NONE) && n_waypoints_[route_] > 0)
        {
          final_waypoint_ = &waypoints_[route_][n_waypoints_[route_]-1];
          // Allow continuation from NAV_MODE_HOLD.
          if ((current_waypoint_ >= final_waypoint_) || (current_waypoint_ <
            &waypoints_[route_][0]))
          {
            current_waypoint_ = &waypoints_[route_][0];
          }
          SetTargetPosition(current_waypoint_);
          target_heading_ = current_waypoint_->target_heading;
          radius_squared = current_waypoint_->radius * current_waypoint_->radius;
          waypoint_reached = 0;
          mode_ = NAV_MODE_AUTO;
        }
        else
        {
          mode_ = NAV_MODE_HOLD;
        }
        break;
      }
      case NAV_MODE_HOME:
      {
        // Set N & E for the first waypoint to current position.
        if (VisionStatus())
        {
          memcpy(go_home_[0].target_position, VisionPositionVector(),
            sizeof(float) * 2);
        }
        else if (UBXStatus())
        {
          UBXToMeters(UBXGeodeticPositionVector(),
            (float *)go_home_[0].target_position);
        }
        else
        {
          memcpy(go_home_[0].target_position, PositionVector(),
            sizeof(float) * 2);
        }

        current_waypoint_ = &go_home_[0];
        final_waypoint_ = &go_home_[2];
        SetTargetPosition(current_waypoint_);
        target_heading_ = current_waypoint_->target_heading;
        radius_squared = current_waypoint_->radius * current_waypoint_->radius;
        waypoint_reached = 0;
        mode_ = NAV_MODE_HOME;
        break;
      }
      default:
      {
        mode_ = RequestedNavMode();
        break;
      }
    }

    if (mode_ == NAV_MODE_HOLD)
    {
      if (VisionStatus())
      {
        Vector3Copy(VisionPositionVector(), target_position_);
      }
      else if (UBXStatus())
      {
        UBXToMeters(UBXGeodeticPositionVector(), target_position_);
        target_position_[D_WORLD_AXIS] = -PressureAltitude();
      }
      else
      {
        Vector3Copy(PositionVector(), target_position_);
      }
      target_heading_ = HeadingAngle();
    }
  }

  // Compute the deviation from that active waypoint.
  Vector3Subtract(PositionVector(), target_position_, delta_postion_);
  delta_heading_ = WrapToPlusMinusPi(HeadingAngle() - target_heading_);

  // Waypoint switching logic.
  if (mode_ == NAV_MODE_AUTO || mode_ == NAV_MODE_HOME)
  {
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
    }
  }

  // Active sensor logic (slight preference for vision).
  if ((mode_ == NAV_MODE_AUTO && current_waypoint_->type & WP_TYPE_BIT_VISION)
    || (mode_ != NAV_MODE_AUTO && VisionStatus()))
  {
#ifdef ALWAYS_USE_MAGNETOMETER
    active_nav_sensors_ = SENSOR_BIT_VISION | SENSOR_BIT_LSM303DL;
#else
    active_nav_sensors_ = SENSOR_BIT_VISION;
#endif
  }
  else if ((mode_ == NAV_MODE_AUTO) || (mode_ != NAV_MODE_AUTO && UBXStatus()))
  {
    active_nav_sensors_ = SENSOR_BIT_UBLOX | SENSOR_BIT_LSM303DL;
  }

  UpdateNavigationToFlightCtrl();
}

// -----------------------------------------------------------------------------
void SetGeodeticHome(void)
{
  memcpy(geodetic_home_, UBXGeodeticPositionVector(), sizeof(geodetic_home_));

  ubx_longitude_to_meters_ = UBX_LATITUDE_TO_METERS
    * cos((float)geodetic_home_[LATITUDE] * 1.0e-7 * M_PI / 180.0);
}

// -----------------------------------------------------------------------------
void UBXToMeters(const int32_t ubx_geodetic[2], float ne[2])
{
    ne[N_WORLD_AXIS] = (float)(ubx_geodetic[LATITUDE]
      - geodetic_home_[LATITUDE]) * UBX_LATITUDE_TO_METERS;
    ne[E_WORLD_AXIS] = (float)(ubx_geodetic[LONGITUDE]
      - geodetic_home_[LONGITUDE]) * ubx_longitude_to_meters_;
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
