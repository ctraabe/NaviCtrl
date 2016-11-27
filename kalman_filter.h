#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_


#include "constants.h"

#include <inttypes.h>


// =============================================================================
// Accessors:

float KalmanVelocity(enum WorldAxes axis);

// -----------------------------------------------------------------------------
const float * KalmanVelocityVector(void);

// =============================================================================
// Public functions:

// This function combines both the Kalman time update and the accelerometer
// measurement update (since they both rely on only the accelerometer).
void KalmanAccelerometerUpdate(void);

// -----------------------------------------------------------------------------
// This function takes an NED velocity measurement and error covariance and 
// computes an update to the state estimate.
void KalmanVelocityMeasurementUpdate(const float velocity[3], const float r[3]);

// -----------------------------------------------------------------------------
void KalmanVisionUpdateFromPosition(void);


#endif // KALMAN_FILTER_H_