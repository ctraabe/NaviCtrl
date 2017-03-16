#include "obstacle_avoidance.h"

#include <math.h>

#include "constants.h"
#include "navigation.h"
#include "vision.h"


// =============================================================================
// Private data:

#define AVOIDANCE_DISTANCE (3.0)

#define CAMERA_FOV (120.0 * M_PI / 180.0)
#define BIN_FOV (CAMERA_FOV / (float)N_BINS)

#define NARROW_ROI (AVOIDANCE_DISTANCE * tan(BIN_FOV))
#define WIDE_ROI (AVOIDANCE_DISTANCE * tan(2.0 * BIN_FOV))

#define BIN_BOUNDARY_ANGLE(i) (((float)i / (float)N_BINS - 1.0) * CAMERA_FOV)
#define BIN_BOUNDARY_TRANSFORM(i, axis) (axis == X_BODY_AXIS ? \
  __builtin_cos(BIN_BOUNDARY_ANGLE(i)) : __builtin_sin(BIN_BOUNDARY_ANGLE(i)))

static const float bin_boundary_angles[2 * N_BINS + 1] = {
  BIN_BOUNDARY_ANGLE(0), BIN_BOUNDARY_ANGLE(1), BIN_BOUNDARY_ANGLE(2),
  BIN_BOUNDARY_ANGLE(3), BIN_BOUNDARY_ANGLE(4), BIN_BOUNDARY_ANGLE(5),
  BIN_BOUNDARY_ANGLE(6), BIN_BOUNDARY_ANGLE(7), BIN_BOUNDARY_ANGLE(8),
  BIN_BOUNDARY_ANGLE(9), BIN_BOUNDARY_ANGLE(10), BIN_BOUNDARY_ANGLE(11),
  BIN_BOUNDARY_ANGLE(12), BIN_BOUNDARY_ANGLE(13), BIN_BOUNDARY_ANGLE(14),
  BIN_BOUNDARY_ANGLE(15), BIN_BOUNDARY_ANGLE(16), BIN_BOUNDARY_ANGLE(17),
  BIN_BOUNDARY_ANGLE(18), BIN_BOUNDARY_ANGLE(19), BIN_BOUNDARY_ANGLE(20),
#if N_BINS > 10
  BIN_BOUNDARY_ANGLE(21), BIN_BOUNDARY_ANGLE(22),
  BIN_BOUNDARY_ANGLE(23), BIN_BOUNDARY_ANGLE(24),
#endif
#if N_BINS > 12
  BIN_BOUNDARY_ANGLE(25), BIN_BOUNDARY_ANGLE(26),
  BIN_BOUNDARY_ANGLE(27), BIN_BOUNDARY_ANGLE(28),
#endif
#if N_BINS > 14
  BIN_BOUNDARY_ANGLE(29), BIN_BOUNDARY_ANGLE(30),
  BIN_BOUNDARY_ANGLE(31), BIN_BOUNDARY_ANGLE(32),
#endif
};

static const float bin_boundary_transforms[2 * N_BINS + 1][2] = {
  { BIN_BOUNDARY_TRANSFORM(0, 0), BIN_BOUNDARY_TRANSFORM(0, 1) },
  { BIN_BOUNDARY_TRANSFORM(1, 0), BIN_BOUNDARY_TRANSFORM(1, 1) },
  { BIN_BOUNDARY_TRANSFORM(2, 0), BIN_BOUNDARY_TRANSFORM(2, 1) },
  { BIN_BOUNDARY_TRANSFORM(3, 0), BIN_BOUNDARY_TRANSFORM(3, 1) },
  { BIN_BOUNDARY_TRANSFORM(4, 0), BIN_BOUNDARY_TRANSFORM(4, 1) },
  { BIN_BOUNDARY_TRANSFORM(5, 0), BIN_BOUNDARY_TRANSFORM(5, 1) },
  { BIN_BOUNDARY_TRANSFORM(6, 0), BIN_BOUNDARY_TRANSFORM(6, 1) },
  { BIN_BOUNDARY_TRANSFORM(7, 0), BIN_BOUNDARY_TRANSFORM(7, 1) },
  { BIN_BOUNDARY_TRANSFORM(8, 0), BIN_BOUNDARY_TRANSFORM(8, 1) },
  { BIN_BOUNDARY_TRANSFORM(9, 0), BIN_BOUNDARY_TRANSFORM(9, 1) },
  { BIN_BOUNDARY_TRANSFORM(10, 0), BIN_BOUNDARY_TRANSFORM(10, 1) },
  { BIN_BOUNDARY_TRANSFORM(11, 0), BIN_BOUNDARY_TRANSFORM(11, 1) },
  { BIN_BOUNDARY_TRANSFORM(12, 0), BIN_BOUNDARY_TRANSFORM(12, 1) },
  { BIN_BOUNDARY_TRANSFORM(13, 0), BIN_BOUNDARY_TRANSFORM(13, 1) },
  { BIN_BOUNDARY_TRANSFORM(14, 0), BIN_BOUNDARY_TRANSFORM(14, 1) },
  { BIN_BOUNDARY_TRANSFORM(15, 0), BIN_BOUNDARY_TRANSFORM(15, 1) },
  { BIN_BOUNDARY_TRANSFORM(16, 0), BIN_BOUNDARY_TRANSFORM(16, 1) },
  { BIN_BOUNDARY_TRANSFORM(17, 0), BIN_BOUNDARY_TRANSFORM(17, 1) },
  { BIN_BOUNDARY_TRANSFORM(18, 0), BIN_BOUNDARY_TRANSFORM(18, 1) },
  { BIN_BOUNDARY_TRANSFORM(19, 0), BIN_BOUNDARY_TRANSFORM(19, 1) },
  { BIN_BOUNDARY_TRANSFORM(20, 0), BIN_BOUNDARY_TRANSFORM(20, 1) },
#if N_BINS > 10
  { BIN_BOUNDARY_TRANSFORM(21, 0), BIN_BOUNDARY_TRANSFORM(21, 1) },
  { BIN_BOUNDARY_TRANSFORM(22, 0), BIN_BOUNDARY_TRANSFORM(22, 1) },
  { BIN_BOUNDARY_TRANSFORM(23, 0), BIN_BOUNDARY_TRANSFORM(23, 1) },
  { BIN_BOUNDARY_TRANSFORM(24, 0), BIN_BOUNDARY_TRANSFORM(24, 1) },
#endif
#if N_BINS > 12
  { BIN_BOUNDARY_TRANSFORM(25, 0), BIN_BOUNDARY_TRANSFORM(25, 1) },
  { BIN_BOUNDARY_TRANSFORM(26, 0), BIN_BOUNDARY_TRANSFORM(26, 1) },
  { BIN_BOUNDARY_TRANSFORM(27, 0), BIN_BOUNDARY_TRANSFORM(27, 1) },
  { BIN_BOUNDARY_TRANSFORM(28, 0), BIN_BOUNDARY_TRANSFORM(28, 1) },
#endif
#if N_BINS > 14
  { BIN_BOUNDARY_TRANSFORM(29, 0), BIN_BOUNDARY_TRANSFORM(29, 1) },
  { BIN_BOUNDARY_TRANSFORM(30, 0), BIN_BOUNDARY_TRANSFORM(30, 1) },
  { BIN_BOUNDARY_TRANSFORM(31, 0), BIN_BOUNDARY_TRANSFORM(31, 1) },
  { BIN_BOUNDARY_TRANSFORM(32, 0), BIN_BOUNDARY_TRANSFORM(32, 1) },
#endif
};


// =============================================================================
// Private function declarations:

static void ClarifyObstacleData(float sensor_readings[N_BINS],
  float obstacles[N_BINS + 1]);
static uint32_t ObstacleInROI(float obstacles[N_BINS + 1], float roi_width,
  float * roi_clearance);
static uint32_t DeviseAvoidanceWaypoint(float obstacles[N_BINS + 1],
  float roi_clearance[N_BINS + 1], float avoidance_waypoint[2]);


// =============================================================================
// Public functions:

uint32_t AvoidanceUpdateHelper(float sensor_readings[N_BINS],
  enum AvoidanceWidth avoidance_width_enum, float avoidance_waypoint[2])
{
  float obstacles[N_BINS + 1], roi_clearance[N_BINS + 1];

  ClarifyObstacleData(sensor_readings, obstacles);

  float roi_width = avoidance_width_enum == AVOIDANCE_WIDTH_NARROW
    ? NARROW_ROI : WIDE_ROI;

  if (ObstacleInROI(obstacles, roi_width, &roi_clearance[N_BINS/2]))
    return DeviseAvoidanceWaypoint(obstacles, roi_clearance,
      avoidance_waypoint);

  return 0;
}


// =============================================================================
// Private functions:

static void ClarifyObstacleData(float sensor_readings[N_BINS],
  float obstacles[N_BINS + 1])
{
  // Assume that obstacles occur on the boundaries, so find the nearest
  // possible obstacle along each boundary.
  obstacles[0] = sensor_readings[0];
  obstacles[N_BINS] = sensor_readings[N_BINS - 1];
  for (size_t i = N_BINS - 1; i--; )
  {
    obstacles[i+1] = sensor_readings[i] < sensor_readings[i+1]
      ? sensor_readings[i] : sensor_readings[i+1];
  }
}

// -----------------------------------------------------------------------------
static uint32_t ObstacleInROI(float obstacles[N_BINS + 1], float roi_width,
  float * roi_clearance)
{
  // Find the distance to the next waypoint.
  const float distance_to_waypoint = sqrt(NavDeltaPosition(N_WORLD_AXIS)
    * NavDeltaPosition(N_WORLD_AXIS) + NavDeltaPosition(E_WORLD_AXIS)
    * NavDeltaPosition(E_WORLD_AXIS));

  // Set the length of the ROI according to the distance the waypoint.
  const float roi_length = distance_to_waypoint > AVOIDANCE_DISTANCE - 0.5 ?
    AVOIDANCE_DISTANCE : distance_to_waypoint + 0.5;

  // Find if any obstacles lie within the ROI.
  *roi_clearance = 99.9;  // Assume the maximum clearance
  for (size_t i = N_BINS + 1; i--; )
  {
    if (fabs(obstacles[i] * bin_boundary_transforms[N_BINS/2+i][1]) < roi_width)
    {
      float temp = obstacles[i] * bin_boundary_transforms[N_BINS/2+i][0];
      if (temp < *roi_clearance) *roi_clearance = temp;
    }
  }

  // Just return of there aren't any obstacles in the ROI.
  return *roi_clearance < roi_length;
}

// -----------------------------------------------------------------------------
static uint32_t DeviseAvoidanceWaypoint(float obstacles[N_BINS + 1],
  float roi_clearance[N_BINS + 1], float avoidance_waypoint[2])
{
  // Realign the ROI along each of the bin boundaries and find the nearest
  // encroaching obstacle in each case.
  float roi_clearance_x[N_BINS + 1];
  for (uint32_t i = N_BINS + 1; i--; )
  {
    // Skip the current heading, which is hopefully not the best.
    if (i == N_BINS / 2) continue;

    roi_clearance[i] = 99.9;
    for (uint32_t j = N_BINS + 1; j--; )
    {
      if (fabs(obstacles[j] * bin_boundary_transforms[N_BINS-i+j][1]) < WIDE_ROI)
      {
        float temp = obstacles[j] * bin_boundary_transforms[N_BINS-i+j][0];
        if (temp < roi_clearance[i]) roi_clearance[i] = temp;
      }
    }
    // Compute the clearance along the x-body axis (since the vehicle should
    // be pointed toward the target waypoint).
    roi_clearance_x[i] = roi_clearance[i] * bin_boundary_transforms[N_BINS/2+i][0];
  }

  // Starting from the center and moving outward, find the rotated ROI that
  // gives the best clearance from obstacles. Stop searching if a region is
  // found with clearance that is 1 m beyond the obstacle that triggered the
  // search.
  float max_clearance = 0;
  size_t i_max_clearance = N_BINS / 2;
  for (size_t i = 1; i <= N_BINS / 2; ++i)
  {
    if (roi_clearance_x[N_BINS/2-i] > max_clearance)
    {
      max_clearance = roi_clearance_x[N_BINS/2-i];
      i_max_clearance = N_BINS / 2 - i;
    }
    if (roi_clearance_x[N_BINS/2+i] > max_clearance)
    {
      max_clearance = roi_clearance_x[N_BINS/2+i];
      i_max_clearance = N_BINS / 2 + i;
    }
    if (max_clearance > roi_clearance[N_BINS/2] + 0.5) break;
  }

  // TODO: pre-compute the tangent below.
  avoidance_waypoint[X_BODY_AXIS] = roi_clearance_x[i_max_clearance] - 1.5;
  float temp = roi_clearance[N_BINS/2] + 0.5;
  if (avoidance_waypoint[X_BODY_AXIS] > temp)
    avoidance_waypoint[X_BODY_AXIS] = temp;
  avoidance_waypoint[Y_BODY_AXIS] = avoidance_waypoint[X_BODY_AXIS]
    * tan(bin_boundary_angles[N_BINS/2+i_max_clearance]);

  return i_max_clearance + 1;
}
