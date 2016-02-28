#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_


// =============================================================================
// Accessors:

const float * KalmanPosition(void);

// -----------------------------------------------------------------------------
const float * KalmanQuat(void);

// -----------------------------------------------------------------------------
const float * KalmanVelocity(void);


// =============================================================================
// Public functions:

void KalmanTimeUpdate(const float gyro[3], const float accelerometer[3]);

// -----------------------------------------------------------------------------
void KalmanAccelerometerUpdate(const float accelerometer[3]);

// -----------------------------------------------------------------------------
void KalmanVisionUpdate(const float vision[3]);

// -----------------------------------------------------------------------------
void ResetKalman(void);


#endif // KALMAN_FILTER_H_