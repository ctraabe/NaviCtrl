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

void KalmanTimeUpdate(void);

// -----------------------------------------------------------------------------
void KalmanVisionUpdateVelocity(const float velocity[3]);

// -----------------------------------------------------------------------------
void KalmanVisionUpdateFromPosition(void);


#endif // KALMAN_FILTER_H_