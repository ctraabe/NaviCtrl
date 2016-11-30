// This file combines noisy NED velocity estimates with the IMU to provide a
// cleaner NED velocity estimate. A naive accelerometer bias estimate is also
// included relying on the assumption that long-term- average acceleration is
// zero.
//
// In MATLAB syntax:
// x = [velocity(3); bias(3)]
// u = acceleration(3)
// A = [eye(3), -dt * eye(3); zeros(3), eye(3)]
// B = [dt * eye(3); zeros(3)]
//
// Since it is assumed that long-term-average acceleration is zero, then
// acceleration is itself a measurement of accelerometer bias, although the
// measurement noise covariance should be set very high.
//
// Note that the structure of this system results in block diagonal P and K.
// This property can be used to drastically speed computation. To facilitate
// this, P is stored as 4 separate block diagonals [P11, P12; P21, P22].

#include "kalman_filter.h"

#include <math.h>

#include "flight_ctrl_comms.h"
#include "matrix.h"
#include "quaternion.h"
#include "vector.h"


// =============================================================================
// Private data:

#define Q_ACCELEROMETER (0.05 * GRAVITY_ACCELERATION)
#define Q_ACCELEROMETER_INTEGRAL (Q_ACCELEROMETER * DT / sqrt(2))
#define Q_ACCELEROMETER_BIAS (1e-5)
#define R_ACCELEROMETER_BIAS (1.0)

static float velocity_[3] = { 0.0 };  // m/s
static float bias_[3] = { 0.0 };  // g
static float p_11_[3] = { 0.0 }, p_12_[3] = { 0.0 }, p_21_[3] = { 0.0 },
  p_22_[3] = { 0.0 };


// =============================================================================
// Private function declarations:

static void AccelerometerBiasMeasurementUpdate(const float acceleration[3]);
static void TimeUpdate(const float acceleration[3]);


// =============================================================================
// Accessors:

float KalmanVelocity(enum WorldAxes axis)
{
  return velocity_[axis];
}

// -----------------------------------------------------------------------------
const float * KalmanVelocityVector(void)
{
  return velocity_;
}


// =============================================================================
// Public functions:

// This function combines both the Kalman time update and the accelerometer
// measurement update (since they both rely on only the accelerometer).
void KalmanAccelerometerUpdate(void)
{
  float acceleration_ned[3];  // NED inertial frame
  QuaternionInverseRotateVector((float *)Quat(), (float *)AccelerometerVector(),
    acceleration_ned);

  // Remove acceleration due to gravity.
  acceleration_ned[D_WORLD_AXIS] -= -1.0;

  // Perform a measurement update of the accelerometer bias.
  AccelerometerBiasMeasurementUpdate(acceleration_ned);

  // Time update (integrate acceleration).
  TimeUpdate(acceleration_ned);

  UpdateVelocityToFlightCtrl();
}

// -----------------------------------------------------------------------------
// This function takes an NED velocity measurement and error covariance and 
// computes an update to the state estimate.
void KalmanVelocityMeasurementUpdate(const float velocity[3], const float r[3])
{
  // Notes:
  // H = [eye(3), zeros(3)]
  // Block diagonal P * H^T = [P11; P21]
  // Diagonal H * P * H^T = P11
  // Diagonal H * x = bias;
  // Block diagonal K * H = [K1, zeros(3); K2, zeros(3)]
  // Block diagonal (I - K * H) = [eye(3) - K1, zeros(3); -K2, eye(3)]
  // Block diagonal (I - K * H) * P
  //   = [P11 - K1 * P11, P12 - K1 * P12; P21 - K2 * P11, P22 - K2 * P12]

  // Compute K = P * H^T / (H * P * H^T + R)
  // Note that 3 reciprocals and 6 multiplies should be faster than 6 divides.
  float k_1[3], k_2[3], den[3];
  DiagonalCopy(r, 3, den);
  DiagonalInvertSelf(DiagonalAddToSelf(den, p_11_, 3), 3);
  DiagonalMultiply(p_11_, den, 3, k_1);
  DiagonalMultiply(p_21_, den, 3, k_2);

  // Update the state x = x + K * (z - H * x)
  float residual[3], temp[3];
  Vector3Subtract(velocity, velocity_, residual);
  Vector3AddToSelf(velocity_, DiagonalMultiply(k_1, residual, 3, temp));
  Vector3AddToSelf(bias_, DiagonalMultiply(k_2, residual, 3, temp));

  // Update the estimate covariance P = (I - K * H) * P
  DiagonalSubtractFromSelf(p_11_, DiagonalMultiply(k_1, p_11_, 3, temp), 3);
  DiagonalSubtractFromSelf(p_12_, DiagonalMultiply(k_1, p_12_, 3, temp), 3);
  DiagonalSubtractFromSelf(p_21_, DiagonalMultiply(k_2, p_11_, 3, temp), 3);
  DiagonalSubtractFromSelf(p_22_, DiagonalMultiply(k_2, p_12_, 3, temp), 3);
}


// =============================================================================
// Private functions:

// This function takes an acceleration measurement in g's and computes a naive
// accelerometer bias estimate.
static void AccelerometerBiasMeasurementUpdate(const float acceleration[3])
{
  // Notes:
  // H = [zeros(3), eye(3)]
  // Block diagonal P * H^T = [P12; P22]
  // Diagonal H * P * H^T = P22
  // Diagonal H * x = bias;
  // Block diagonal K * H = [zeros(3), K1; zeros(3), K2]
  // Block diagonal (I - K * H) = [eye(3), -K1; zeros(3), eye(3) - K2]
  // Block diagonal (I - K * H) * P
  //   = [P11 - K1 * P21, P12 - K1 * P22; P21 - K2 * P21, P22 - K2 * P22]

  // Compute K = P * H^T / (H * P * H^T + R)
  // Note that 3 reciprocals and 6 multiplies should be faster than 6 divides.
  float k_1[3], k_2[3], den[3] = { R_ACCELEROMETER_BIAS, R_ACCELEROMETER_BIAS,
    R_ACCELEROMETER_BIAS };
  DiagonalInvertSelf(DiagonalAddToSelf(den, p_22_, 3), 3);
  DiagonalMultiply(p_12_, den, 3, k_1);
  DiagonalMultiply(p_22_, den, 3, k_2);

  // Update the state x = x + K * (z - H * x)
  float residual[3], temp[3];
  Vector3Subtract(acceleration, bias_, residual);
  Vector3AddToSelf(velocity_, DiagonalMultiply(k_1, residual, 3, temp));
  Vector3AddToSelf(bias_, DiagonalMultiply(k_2, residual, 3, temp));

  // Update the estimate covariance P = (I - K * H) * P
  DiagonalSubtractFromSelf(p_11_, DiagonalMultiply(k_1, p_21_, 3, temp), 3);
  DiagonalSubtractFromSelf(p_12_, DiagonalMultiply(k_1, p_22_, 3, temp), 3);
  DiagonalSubtractFromSelf(p_21_, DiagonalMultiply(k_2, p_21_, 3, temp), 3);
  DiagonalSubtractFromSelf(p_22_, DiagonalMultiply(k_2, p_22_, 3, temp), 3);
}

// -----------------------------------------------------------------------------
// This function takes an NED acceleration measurement in g's and computes a
// time update.
static void TimeUpdate(const float acceleration[3])
{
  // Notes:
  // Block diagonal A = [eye(3), -dt * eye(3); zeros(3), eye(3)]
  // Block diagonal A^T = [eye(3), zeros(3); -dt * eye(3), eye(3)]
  // Block diagonal AP = A * P = [P11 - dt * P21, P12 - dt * P22; P21, P22]
  //   (Note that AP21 = P21 and AP22 = P22)
  // Block diagonal AP * A^T = [AP11 - dt * AP12, AP12; AP21 - dt * AP22, AP22]
  //   (Note that APAT12 = AP12 and APAT22 = AP22 = P22)
  // Block diagonal P = [APAT11 + Q_V, APAT12; APAT21, P22 + Q_B]
  //   (Note that P12 = APAT12 = AP12 and P21 = APAT21)

  float temp[3];

  // Convert g's to m/s^2 and multiply by DT for integration to velocity.
  float unbiased_acceleration[3];
  Vector3Subtract(acceleration, bias_, unbiased_acceleration);
  Vector3Scale(unbiased_acceleration, GRAVITY_ACCELERATION * DT, temp);
  Vector3AddToSelf(velocity_, temp);

  // Compute A * P (intermediate).
  float ap_11[3], ap_12[3];
  DiagonalSubtractSelfFrom(DiagonalScale(p_21_, DT, 3, ap_11), p_11_, 3);
  DiagonalSubtractSelfFrom(DiagonalScale(p_22_, DT, 3, ap_12), p_12_, 3);
  float * ap_21 = p_21_;
  float * ap_22 = p_22_;

  // Compute A * P * A^T (intermediate).
  float apat_11[3], apat_21[3];
  DiagonalSubtractSelfFrom(DiagonalScale(ap_12, DT, 3, apat_11), ap_11, 3);
  float * apat_12 = ap_12;
  DiagonalSubtractSelfFrom(DiagonalScale(ap_22, DT, 3, apat_21), ap_21, 3);
  float * apat_22 = ap_22;

  // Update the estimate covariance P = A * P * A^T + Q.
  DiagonalAddToEachElement(apat_11, Q_ACCELEROMETER_INTEGRAL, 3, p_11_);
  DiagonalCopy(apat_12, 3, p_12_);
  DiagonalCopy(apat_21, 3, p_21_);
  DiagonalAddToEachElement(apat_22, Q_ACCELEROMETER_BIAS, 3, p_22_);
}
