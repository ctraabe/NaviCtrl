#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_


// =============================================================================
// Private functions:

void KalmanTimeUpdate(const float gyro[3], const float accelerometer[3]);

// -----------------------------------------------------------------------------
void KalmanAccelerometerUpdate(const float accelerometer[3]);

// -----------------------------------------------------------------------------
void KalmanVisionUpdate(const float vision[3]);


#endif // KALMAN_FILTER_H_