#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <inttypes.h>


// =============================================================================
// Accessors:

float * KalmanVelocityVector(void);


// =============================================================================
// Public functions:

void KalmanTimeUpdate(const float quaternion[4], const float accelerometer[3]);

// -----------------------------------------------------------------------------
void KalmanVisionUpdate(const float vision_position[3], float vision_dt,
  int16_t vision_status);


#endif // KALMAN_FILTER_H_