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
void KalmanVisionUpdate(void);


#endif // KALMAN_FILTER_H_