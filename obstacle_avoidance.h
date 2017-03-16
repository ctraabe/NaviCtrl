#ifndef OBSTACLE_AVOIDANCE_H_
#define OBSTACLE_AVOIDANCE_H_


#include <inttypes.h>


#define N_BINS (12)  // Must be an even number between 10 and 16


enum AvoidanceMode {
  AVOIDANCE_MODE_NOMINAL,
  AVOIDANCE_MODE_NOMINAL_ROTATION,
  AVOIDANCE_MODE_FINISHED_ROTATING,
  AVOIDANCE_MODE_AVOIDANCE,
  AVOIDANCE_MODE_AVOIDANCE_ROTATIOIN,
};

enum AvoidanceWidth {
  AVOIDANCE_WIDTH_NARROW,
  AVOIDANCE_WIDTH_WIDE,
};


// =============================================================================
// Accessors:


// =============================================================================
// Public functions:

uint32_t AvoidanceUpdateHelper(float sensor_readings[N_BINS],
  enum AvoidanceWidth avoidance_width_enum, float avoidance_waypoint[2]);


#endif  // OBSTACLE_AVOIDANCE_H_
