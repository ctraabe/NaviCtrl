#include "navigation.h"

#include <math.h>
#include <stdlib.h>

#include "custom_math.h"
#include "ff.h"  // from libfatfs
#include "flight_ctrl_comms.h"
#include "sd_card.h"
#include "timing.h"
#include "uart.h"
#include "vector.h"
#ifdef VISION
  #include "vision.h"
#endif


// =============================================================================
// Private data:

#define N_ROUTES (3)
#define MAX_WAYPOINTS (32)

struct Waypoint {
  float target_position[3];
  float transit_speed;
  float radius;
  float target_heading;
  float heading_rate;
  float heading_range;
  uint32_t wait_ms;
};

enum RouteNumber {
  ROUTE_1 = 0,
  ROUTE_2 = 1,
  ROUTE_3 = 2,
};

static uint32_t n_waypoints_[N_ROUTES] = { 0, 0, 0 };
static struct Waypoint waypoints_[N_ROUTES][MAX_WAYPOINTS] = { 0 };
static enum NavMode mode_ = NAV_MODE_OFF;
static enum NavError nav_error_ = NAV_ERROR_NONE;
static float target_position_[3] = { 0.0 }, delta_postion_[3] = { 0.0 };
static float delta_heading_ = 0.0;
static const struct Waypoint * current_waypoint_ = &waypoints_[ROUTE_1][0];
static const struct Waypoint * final_waypoint_ = &waypoints_[ROUTE_1][0];


// =============================================================================
// Accessors:

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
  return current_waypoint_->target_heading;
}

// -----------------------------------------------------------------------------
float TargetPosition(enum WorldAxes axis)
{
  return target_position_[axis];
}

// -----------------------------------------------------------------------------
float TransitSpeed(void)
{
  return current_waypoint_->transit_speed;
}


// =============================================================================
// Public functions:

// Try to load waypoints from WP_LIST.CSV on the SD card (if inserted).
void NavigationInit(void)
{
  if (!SDCardFSMounted()) return;

  FIL file;
  char filename[12] = "WP_LIST.CSV", line[128];
  if (f_open(&file, filename, FA_READ) != FR_OK)
  {
    UARTPrintf("Failed to open %s", filename);
    return;
  }

  UARTPrintf("Opened file %s", filename);

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
        UARTPrintf("navigation: invalid route number in WP_LIST.CSV line %i",
          line_number);
        goto CLOSE_AND_RETURN;
      }
      continue;  // Skip to the next line
    }

    // Loop through the comma separated data in the line.
    char * line_ptr = line;
    float * data_ptr = (float *)waypoint_ptr;
    uint32_t data_index = 0;
    for (;;)
    {
      if ((*line_ptr > 0x1F) && (*line_ptr != ','))  // Data
      {
        // Check that number of waypoints does not exceed the maximum.
        if (n_waypoints_[route_index] == MAX_WAYPOINTS)
        {
          nav_error_ = NAV_ERROR_SD_WAYPOINT_COUNT;
          UARTPrintf("navigation: too many waypoints in WP_LIST.CSV line %i",
            line_number);
          goto CLOSE_AND_RETURN;
        }

        // The first 8 entries are floats, then a 32-bit integer.
        if (data_index < 8)
          data_ptr[data_index] = atof(line_ptr);
        else if (data_index == 8)
          waypoint_ptr->wait_ms = atoi(line_ptr);
        data_index++;
        // Move the pointer past the next comma (if possible).
        do { line_ptr++; } while ((*line_ptr > 0x1F) && (*line_ptr != ','));
      }

      if (*line_ptr == 0)  // End of line
      {
        // Ignore blank lines
        if (data_index == 0) continue;  // Skip to the next line

        // Check that the correct number of data entries were read.
        if (data_index == 9)
        {
          // Post processing (convert degrees to radians)
          waypoint_ptr->target_heading *= M_PI / 180.0;
          waypoint_ptr->heading_rate *= M_PI / 180.0;
          waypoint_ptr->heading_range *= M_PI / 180.0;

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
          UARTPrintf("navigation: incompatible number of entries in WP_LIST.CSV"
            " line %i", line_number);
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
    UARTPrintf("Route %i:", i + 1);
    for (uint32_t j = 0; j < n_waypoints_[i]; j++)
    {
      UARTPrintf("%02i: (%+06.2f %+06.2f %+06.2f) %4.2f %4.2f %+04.0f %03.0f"
        " %03.0f %i", j + 1,
        waypoints_[i][j].target_position[0],
        waypoints_[i][j].target_position[1],
        waypoints_[i][j].target_position[2],
        waypoints_[i][j].transit_speed,
        waypoints_[i][j].radius,
        waypoints_[i][j].target_heading * 180.0 / M_PI,
        waypoints_[i][j].heading_rate * 180.0 / M_PI,
        waypoints_[i][j].heading_range * 180.0 / M_PI,
        waypoints_[i][j].wait_ms);
    }
  }

  UARTPrintf("End of %s", filename);

  CLOSE_AND_RETURN:
  f_close(&file);
}

// -----------------------------------------------------------------------------
void UpdateNavigation(void)
{
  static float radius_squared = 1.0;
  static uint32_t next_waypoint_time = 0, waypoint_reached = 0;  // , baro_reset = 0;

  const float * current_position = VisionPositionVector();

  if (RequestedNavMode() != mode_)
  {
    if (RequestedNavMode() == NAV_MODE_AUTO)
    {
      uint32_t route = RequestedNavRoute();
      if ((nav_error_ == NAV_ERROR_NONE) && n_waypoints_[route] > 0)
      {
        current_waypoint_ = &waypoints_[route][0];
        final_waypoint_ = &waypoints_[route][n_waypoints_[route]-1];
        Vector3Copy(current_waypoint_->target_position, target_position_);
        radius_squared = current_waypoint_->radius * current_waypoint_->radius;
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
      Vector3Copy(current_position, target_position_);
    }
  }

  Vector3Subtract(current_position, target_position_, delta_postion_);
  delta_heading_ = WrapToPlusMinusPi(VisionHeading()
    - current_waypoint_->target_heading);

  if ((RequestedNavMode() == NAV_MODE_AUTO))
  {
    if (!waypoint_reached
      && (Vector3NormSquared(delta_postion_) < radius_squared)
      && (fabs(delta_heading_) < current_waypoint_->heading_range))
    {
      waypoint_reached = 1;
      next_waypoint_time = GetTimestampMillisFromNow(current_waypoint_->wait_ms)
        - 1;
    }
    if (waypoint_reached && (current_waypoint_ < final_waypoint_)
      && TimestampInPast(next_waypoint_time))
    {
      waypoint_reached = 0;
      current_waypoint_++;
      Vector3Copy(current_waypoint_->target_position, target_position_);
      radius_squared = current_waypoint_->radius * current_waypoint_->radius;
    }
  }
}
