#include "navigation.h"

#include "flight_ctrl_comms.h"
#include "kalman_filter.h"
#include "vector.h"


// =============================================================================
// Private data:

#define WAYPOINT_RADIUS (0.50)  // Inside radius indicates waypoint reached
#define WAYPOINT_RADIUS_SQUARED (WAYPOINT_RADIUS * WAYPOINT_RADIUS)
/*
#define ROUTE_0_N_WAYPOINTS (3)
static const float route0[ROUTE_0_N_WAYPOINTS][4] = {
  { +0.0, +0.0, -1.0, 0.0 },
  { +2.0, +0.0, -1.0, 0.0 },
  { +0.0, +0.0, -1.0, 0.0 },
};

#define ROUTE_1_N_WAYPOINTS (5)
static const float route1[ROUTE_1_N_WAYPOINTS][4] = {
  { +0.0, +0.0, -1.0, 0.0 },
  { +2.0, +0.0, -1.0, 0.0 },
  { +2.0, +3.0, -1.0, 0.0 },
  { +2.0, +0.0, -1.0, 0.0 },
  { +0.0, +0.0, -1.0, 0.0 },
};

#define ROUTE_2_N_WAYPOINTS (7)
static const float route2[ROUTE_2_N_WAYPOINTS][4] = {
  { +0.0, +0.0, -1.0, 0.0 },
  { +2.0, +0.0, -1.0, 0.0 },
  { +2.0, +3.0, -1.0, 0.0 },
  { +6.7, +3.0, -1.0, 0.0 },
  { +2.0, +3.0, -1.0, 0.0 },
  { +2.0, +0.0, -1.0, 0.0 },
  { +0.0, +0.0, -1.0, 0.0 },
};
*/
#define ROUTE_0_N_WAYPOINTS (9)
static const float route0[ROUTE_0_N_WAYPOINTS][4] = {
  { +0.0, +0.0, -1.0, 0.0 },
  { +2.0, +0.0, -1.0, 0.0 },
  { +2.0, +3.0, -1.0, 0.0 },
  { +6.7, +3.0, -1.0, 0.0 },
  { +6.7, -1.0, -1.0, 0.0 },
  { +6.7, +3.0, -1.0, 0.0 },
  { +2.0, +3.0, -1.0, 0.0 },
  { +2.0, +0.0, -1.0, 0.0 },
  { +0.0, +0.0, -1.0, 0.0 },
};

#define ROUTE_1_N_WAYPOINTS (3)
static const float route1[ROUTE_1_N_WAYPOINTS][4] = {
  { +0.0, +0.0, -1.0, 0.0 },
  { +2.0, +0.0, -2.0, 0.0 },
  { +0.0, +0.0, -1.0, 0.0 },
};

#define ROUTE_2_N_WAYPOINTS (6)
static const float route2[ROUTE_2_N_WAYPOINTS][4] = {
  { +0.0, +0.0, -1.0, 0.0 },
  { +2.0, +0.0, -2.0, 0.0 },
  { +6.7, +0.0, -2.0, 0.0 },
  { +6.7, +0.0, -1.0, 0.0 },
  { +6.7, +0.0, +0.0, 0.0 },
  { +6.7, +0.0, -1.0, 0.0 },
};

static enum NavMode mode_ = NAV_MODE_OFF;
static float target_position_[3] = { 0.0 }, delta_postion_[3] = { 0.0 };
static const float * final_waypoint_ = route1[0];
static const float * current_waypoint_ = route1[0];


// =============================================================================
// Private function declarations:

static void GetNexWaypoint(void);


// =============================================================================
// Accessors:

const float * NavDeltaPosition(void)
{
  return delta_postion_;
}

// -----------------------------------------------------------------------------
enum NavMode NavMode(void)
{
  return mode_;
}


// =============================================================================
// Public functions:

void UpdateNavigation(void)
{
  const float * current_position = KalmanPosition();

  if (RequestedNavMode() != mode_)
  {
    if (RequestedNavMode() == NAV_MODE_AUTO)
    {
      // Select the route.
      switch (RequestedNavRoute())
      {
        case 0:
          current_waypoint_ = route0[0];
          final_waypoint_ = route0[ROUTE_0_N_WAYPOINTS-1];
          break;
        case 1:
          current_waypoint_ = route1[0];
          final_waypoint_ = route1[ROUTE_1_N_WAYPOINTS-1];
          break;
        case 2:
          current_waypoint_ = route2[0];
          final_waypoint_ = route2[ROUTE_2_N_WAYPOINTS-1];
          break;
      }

      Vector3Copy(current_waypoint_, target_position_);
    }
    else if (RequestedNavMode() == NAV_MODE_HOLD)
    {
      Vector3Copy(current_position, target_position_);
    }
  }

  Vector3Subtract(current_position, target_position_, delta_postion_);

  if ((RequestedNavMode() == NAV_MODE_AUTO)
    && (Vector3NormSquared(delta_postion_) < WAYPOINT_RADIUS_SQUARED))
  {
    GetNexWaypoint();
  }

  mode_ = RequestedNavMode();
}

// -----------------------------------------------------------------------------
static void GetNexWaypoint(void)
{
  if (current_waypoint_ < final_waypoint_)
  {
    current_waypoint_ += 4;
    Vector3Copy(current_waypoint_, target_position_);
  }
}
