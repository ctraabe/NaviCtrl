#include "navigation.h"

#include <math.h>

#include "custom_math.h"
#include "flight_ctrl_comms.h"
#include "kalman_filter.h"
#include "timing.h"
#include "vector.h"
#ifdef VISION
  #include "vision.h"
#endif


// =============================================================================
// Private data:

struct Waypoint {
  float target_position[3];
  float transit_speed;
  float radius;
  float target_heading;
  float heading_rate;
  float heading_range;
  uint32_t wait_ms;
};

#define ROUTE_0_N_WAYPOINTS (8)
static const struct Waypoint route0[ROUTE_0_N_WAYPOINTS] = {
  { { +0.0, +0.0, -1.5 }, +0.75, +0.35, +0.0, +0.2, +0.2, 0 },
  { { +2.0, +0.0, -1.5 }, +0.75, +0.50, +0.0, +0.2, +0.2, 0 },
  { { +2.0, +3.5, -1.5 }, +0.75, +0.50, +0.0, +0.2, +0.2, 0 },
  { { +6.6, +3.5, -1.5 }, +0.75, +0.35, +0.0, +0.2, +0.2, 5000 },
  { { +2.0, +3.5, -1.5 }, +0.75, +0.35, +0.0, +0.2, +0.2, 1000 },
  { { +2.0, +0.0, -1.5 }, +0.75, +0.50, +0.0, +0.2, +0.2, 0 },
  { { +0.0, +0.0, -1.5 }, +0.75, +0.35, +0.0, +0.2, +0.2, 1000 },
  { { +0.0, +0.0, +1.5 }, +0.70, +0.35, +0.0, +0.2, +0.2, 0 },
};

#define ROUTE_1_N_WAYPOINTS (6)
static const struct Waypoint route1[ROUTE_1_N_WAYPOINTS] = {
  { { +0.0, +0.0, -1.5 }, +0.75, +0.35, +0.0, +0.2, +0.2, 0 },
  { { +2.0, +0.0, -2.5 }, +0.75, +0.35, +0.0, +0.2, +0.2, 0 },
  { { +6.6, +0.0, -2.5 }, +0.75, +0.35, +0.0, +0.2, +0.2, 5000 },
  { { +2.0, +0.0, -2.5 }, +0.75, +0.35, +0.0, +0.2, +0.2, 0 },
  { { +0.0, +0.0, -1.5 }, +0.75, +0.35, +0.0, +0.2, +0.2, 2000 },
  { { +0.0, +0.0, +1.5 }, +0.70, +0.35, +0.0, +0.2, +0.2, 0 },
};

#define ROUTE_2_N_WAYPOINTS (13)
static const struct Waypoint route2[ROUTE_2_N_WAYPOINTS] = {
  { { +0.0, +0.0, -1.5 }, +0.75, +0.35, +0.0, +0.2, +0.2, 0 },
  { { +2.0, +0.0, -1.5 }, +0.75, +0.50, +0.0, +0.2, +0.2, 0 },
  { { +2.0, +3.5, -1.5 }, +0.75, +0.50, +0.0, +0.2, +0.2, 0 },
  { { +6.6, +3.5, -1.5 }, +0.75, +0.50, +0.0, +0.2, +0.2, 0 },
  { { +6.6, -0.5, -1.5 }, +0.75, +0.50, +0.0, +0.2, +0.2, 0 },
  { { +9.9, -0.5, -1.5 }, +0.75, +0.50, +0.0, +0.2, +0.2, 0 },
  { { +11.2, -0.5, -1.5 }, +0.5, +0.50, +0.0, +0.2, +0.2, 3000 },
  { { +11.2, +3.5, -1.5 }, +0.5, +0.50, +0.0, +0.2, +0.2, 5000 },
  { { +11.2, -0.5, -1.5 }, +0.5, +0.50, +0.0, +0.2, +0.2, 0 },
  { { +6.6, -0.5, -2.5 }, +0.75, +0.35, +0.0, +0.2, +0.2, 0 },
  { { +2.0, +0.0, -2.5 }, +0.75, +0.35, +0.0, +0.2, +0.2, 0 },
  { { +0.0, +0.0, -1.5 }, +0.50, +0.35, +0.0, +0.2, +0.2, 2000 },
  { { +0.0, +0.0, +1.5 }, +0.70, +0.35, +0.0, +0.2, +0.2, 0 },
};

static enum NavMode mode_ = NAV_MODE_OFF;
static float target_position_[3] = { 0.0 }, delta_postion_[3] = { 0.0 };
static float delta_heading_ = 0.0;
static const struct Waypoint * current_waypoint_ = &route0[0];
static const struct Waypoint * final_waypoint_ = &route0[0];


// =============================================================================
// Accessors:

float HeadingRate(void)
{
  return current_waypoint_->heading_rate;
}

// -----------------------------------------------------------------------------
const float * NavDeltaPosition(void)
{
  return delta_postion_;
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
const float * TargetPosition(void)
{
  return target_position_;
}

// -----------------------------------------------------------------------------
float TransitSpeed(void)
{
  return current_waypoint_->transit_speed;
}


// =============================================================================
// Public functions:

void UpdateNavigation(void)
{
  static float radius_squared = 1.0;
  static uint32_t next_waypoint_time = 0, waypoint_reached = 0, baro_reset = 0;

  const float * current_position = KalmanPosition();

  if (RequestedNavMode() != mode_)
  {
    if (RequestedNavMode() == NAV_MODE_AUTO)
    {
      // Select the route.
      switch (RequestedNavRoute())
      {
        case 0:
          current_waypoint_ = &route0[0];
          final_waypoint_ = &route0[ROUTE_0_N_WAYPOINTS-1];
          break;
        case 1:
          current_waypoint_ = &route1[0];
          final_waypoint_ = &route1[ROUTE_1_N_WAYPOINTS-1];
          break;
        case 2:
          current_waypoint_ = &route2[0];
          final_waypoint_ = &route2[ROUTE_2_N_WAYPOINTS-1];
          break;
      }
      Vector3Copy(current_waypoint_->target_position, target_position_);
      radius_squared = current_waypoint_->radius * current_waypoint_->radius;
    }
    else if (RequestedNavMode() == NAV_MODE_HOLD)
    {
      Vector3Copy(current_position, target_position_);
    }
#ifdef VISION
    if (!baro_reset && (RequestedNavMode() != NAV_MODE_OFF)
      && (FlightCtrlState() & FC_STATE_BIT_MOTORS_RUNNING))
    {
      baro_reset = 1;
      ResetKalmanBaroAltitudeOffset(FilteredPressureAltitude(),
        VisionPositionVector()[D_WORLD_AXIS]);
    }
#endif
  }

  Vector3Subtract(current_position, target_position_, delta_postion_);
  delta_heading_ = WrapToPlusMinusPi(KalmanHeading()
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

  mode_ = RequestedNavMode();
}
