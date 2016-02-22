#include "navigation.h"

#include "vector.h"
#include "vision.h"


// =============================================================================
// Private data:

#define WAYPOINT_RADIUS (0.25)  // Inside radius indicates waypoint reached
#define WAYPOINT_RADIUS_SQUARED (WAYPOINT_RADIUS * WAYPOINT_RADIUS)
#define MAX_N_WAYPOINTS (16)

enum NavigationMode mode_ = NAVIGATION_MODE_OFF;

static float target_position_[3] = { 0.0 }, vector_to_target_[3] = { 0.0 };
static float waypoints[MAX_N_WAYPOINTS][3] = {
  { 0.0, 0.0, 1.0 },
  { 1.0, 0.0, 1.0 },
  { 0.0, 1.0, 1.0 },
};
static float * final_waypoint_ = waypoints[2];
static float * current_waypoint_ = waypoints[0];


// =============================================================================
// Private function declarations:

static void GetNexWaypoint(void);


// =============================================================================
// Public functions:

void UpdateNavigation(void)
{
  static enum NavigationMode mode_pv = NAVIGATION_MODE_OFF;
  const float * current_position = VisionPositionVector();

  if (mode_ != mode_pv)
  {
    if (mode_ == NAVIGATION_MODE_WAYPOINT)
    {
      Vector3Copy(current_waypoint_, target_position_);
    }
    else if (mode_ == NAVIGATION_MODE_HOLD)
    {
      Vector3Copy(current_position, target_position_);
    }
  }

  Vector3Subtract(target_position_, current_position, vector_to_target_);

  if ((mode_ == NAVIGATION_MODE_WAYPOINT)
    && (Vector3NormSquared(vector_to_target_) < WAYPOINT_RADIUS_SQUARED))
  {
    GetNexWaypoint();
  }

  mode_pv = mode_;
}

// -----------------------------------------------------------------------------
static void GetNexWaypoint(void)
{
  if (current_waypoint_ < final_waypoint_) current_waypoint_ += 3;
}
