#include "kalman_filter.h"

#include <stdio.h>

#include "attitude.h"
#include "constants.h"
#include "matrix.h"
#include "quaternion.h"
#include "vector.h"


// =============================================================================
// Private data:

// Vector and Matrix dimensions
#define X_DIM (10)  // Length of state vector x. [q, v_i, r_i]
#define U_DIM (6)  // Length of input u. [gyro; accel]
#define P_DIM (9)  // Size of P (square) matrix
#define Z_DIM_MAX (3)  // Maximum of z (for memory allocation)

// Measurement noise covariance
#define G GRAVITY_ACCELERATION
#define KALMAN_Q_PRIME_ALPHA (0)
#define KALMAN_Q_PRIME_VELOCITY (0.001)
#define KALMAN_Q_PRIME_H_POSITION (0.0)
// #define KALMAN_Q_PRIME_V_POSITION (0.0)
#define KALMAN_Q_PRIME_V_POSITION (0.001)
#define KALMAN_SIGMA_ACCELEROMETER_X (0.005)
#define KALMAN_SIGMA_ACCELEROMETER_Y (0.005)
#define KALMAN_SIGMA_ACCELEROMETER_Z (0.042)
#define KALMAN_SIGMA_ACCELEROMETER_G (0.25)
#define KALMAN_SIGMA_BARO (0.68)
#define KALMAN_SIGMA_GYRO (0.007)
#define KALMAN_SIGMA_VISION (0.02)

static float heading_ = 0.0;
static float x_[X_DIM] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
static float P_[P_DIM*P_DIM] = { 0.0 };
static float baro_altitude_offset = 0.0;


// =============================================================================
// Accessors:

float KalmanHeading(void)
{
  return heading_;
}

// -----------------------------------------------------------------------------
const float * KalmanPosition(void)
{
  return &x_[7];
}

// -----------------------------------------------------------------------------
const float * KalmanQuat(void)
{
  return &x_[0];
}

// -----------------------------------------------------------------------------
const float * KalmanVelocity(void)
{
  return &x_[4];
}


// =============================================================================
// Private function declarations:

static void TimeUpdate(const float * x_est_prev, const float * P_est_prev,
  const float * gyro, const float * accelerometer, float * x_pred,
  float * P_pred);
static void AccelerometerUpdate(const float * x_pred, const float * P_pred,
  const float * accelerometer, float * x_est, float * P_est);
static void BaroAltitudeUpdate(const float *x_pred, const float *P_pred,
  float baro_altitude, float *x_est, float *P_est);
static void VisionUpdate(const float * x_pred, const float * P_pred,
  const float * vision, float * x_est, float * P_est);
static void MeasurementUpdateCommon(const float * x_pred, const float * P_pred,
  const float * z, float * x_est, float * P_est, int z_dim,
  const float * R_diag, const float * H, const float * predicted_measurement);
static float * MatrixMultiplySkewSymmetric3(const float * A, const float B[3*3],
  size_t A_rows, float * result);
static float * QuaternionToDCM(const float *quat, float *result);
// static float * SkewSymmetric3Transpose(const float A[3*3] , float result[3*3]);
static float * UpdateQuaternion(const float quat[4],
  const float angular_rate[3], float result[4]);
static float * Vector3ToSkewSymmetric3(const float vec[3], float *result);


// =============================================================================
// Public functions:

void KalmanTimeUpdate(const float gyro[3], const float accelerometer[3])
{
  float x_pred[X_DIM];
  TimeUpdate(x_, P_, gyro, accelerometer, x_pred, P_);
  VectorCopy(x_pred, X_DIM, x_);
}

// -----------------------------------------------------------------------------
void KalmanAccelerometerUpdate(const float accelerometer[3])
{
  float x_est[X_DIM];
  float P_est[P_DIM*P_DIM];
  AccelerometerUpdate(x_, P_, accelerometer, x_est, P_est);
  VectorCopy(x_est, X_DIM, x_);
  MatrixCopy(P_est, P_DIM, P_DIM, P_);
}

// -----------------------------------------------------------------------------
void KalmanBaroAltitudeUpdate(float baro_altitude)
{
  float x_est[X_DIM];
  float P_est[P_DIM*P_DIM];
  BaroAltitudeUpdate(x_, P_, baro_altitude, x_est, P_est);
  VectorCopy(x_est, X_DIM, x_);
  MatrixCopy(P_est, P_DIM, P_DIM, P_);
}

// -----------------------------------------------------------------------------
void KalmanVisionUpdate(const float vision[3])
{
  float x_est[X_DIM];
  float P_est[P_DIM*P_DIM];
  VisionUpdate(x_, P_, vision, x_est, P_est);
  VectorCopy(x_est, X_DIM, x_);
  MatrixCopy(P_est, P_DIM, P_DIM, P_);
}

// -----------------------------------------------------------------------------
void ResetKalman(void)
{
  heading_ = 0.0;
  x_[0] = 1.0;
  for (size_t i = 10; --i; ) x_[i] = 0.0;
  for (size_t i = P_DIM * P_DIM; --i; ) P_[i] = 0.0;
  P_[0*9+0] = 1.0e-05;
  P_[1*9+1] = 1.0e-05;
  P_[2*9+2] = 1.0e-05;
}

// -----------------------------------------------------------------------------
void ResetKalmanBaroAltitudeOffset(float baro_altitude, float position_down)
{
  baro_altitude_offset = baro_altitude - (-position_down);
  x_[9] = position_down;
}


// =============================================================================
// Private functions:

static void TimeUpdate(const float * x_est_prev, const float * P_est_prev,
  const float * gyro, const float * accelerometer, float * x_pred,
  float * P_pred)
{
  const float * quat_prev = &x_est_prev[0];
  const float * velocity_prev = &x_est_prev[4];
  const float * position_prev = &x_est_prev[7];

  float * quat_next = &x_pred[0];
  float * velocity_next = &x_pred[4];
  float * position_next = &x_pred[7];

  // Update the quaternion with gyro measurements.
  UpdateQuaternion(quat_prev, gyro, quat_next);
  heading_ = HeadingFromQuaternion(quat_next);

  // Form body to inertial direction-cosine matrix.
  float Cbi[3 * 3], temp[3*3];
  QuaternionToDCM(QuaternionInverse(quat_prev, temp), Cbi);

  // Transform acceleration measured in the body frame to the inertial frame.
  float acceleration[3];  // Specific force measured in the inertial frame
  MatrixMultiply(Cbi, accelerometer, 3, 3, 1, acceleration);
  acceleration[2] += G;  // Remove accelerometer gravity bias

  // Integrate the acceleration to get velocity.
  Vector3Add(velocity_prev, Vector3Scale(acceleration, DT, temp),
    velocity_next);

  // Update position.
  Vector3Add(position_prev, Vector3Scale(velocity_prev, DT, temp),
    position_next);

  // Update error covariance matrix P
  // 1. Discretization of error process model. Calculate Phi and Gamma, where
  //    deltax_next = Phi*deltax_prev + Gamma*deltau + process noise by modeling
  //    error. deltau is the "error input" into the process, which is process
  //    noise by IMU. (deltau=[gyro error; accel error])
  // 2. Definition of Q and Qprime (which are constant throughout the
  //    experiment)
  //    Q: covariance matrix of process noise due to IMU
  //       This can be modeled beforehand.
  //    Qprime: covariance matrix of process noise due to modeling error.
  //            Such noise may be introduced into the system by angular momentum
  //            acting on the body, for example. This term will be used as a
  //            filter design parameter.
  // 3. Propagation of P
  //    P_pred = Phi*P_est_prev*Phi^T + Gamma*Q*Gamma^T + Qprime


  // 3. Propagate P

  // P_pred = Phi*P*Phi^t + Gamma*Q*Gamma^t + Qprime
  float * PhiPPhit = P_pred;  // conserves memory
  {
    float P11[3*3], P12[3*3], P13[3*3];
    float P21[3*3], P22[3*3], P23[3*3];
    float P31[3*3], P32[3*3], P33[3*3];

    SubmatrixCopyToMatrix(P_est_prev, P11, 0, 0, P_DIM, 3, 3);
    SubmatrixCopyToMatrix(P_est_prev, P12, 0, 3, P_DIM, 3, 3);
    SubmatrixCopyToMatrix(P_est_prev, P13, 0, 6, P_DIM, 3, 3);
    SubmatrixCopyToMatrix(P_est_prev, P21, 3, 0, P_DIM, 3, 3);
    SubmatrixCopyToMatrix(P_est_prev, P22, 3, 3, P_DIM, 3, 3);
    SubmatrixCopyToMatrix(P_est_prev, P23, 3, 6, P_DIM, 3, 3);
    SubmatrixCopyToMatrix(P_est_prev, P31, 6, 0, P_DIM, 3, 3);
    SubmatrixCopyToMatrix(P_est_prev, P32, 6, 3, P_DIM, 3, 3);
    SubmatrixCopyToMatrix(P_est_prev, P33, 6, 6, P_DIM, 3, 3);

    // Phi = I + A * dt
    // const float Phi[P_DIM*P_DIM] = {
    //               1,    gyro[2]*DT,   -gyro[1]*DT,    0,    0,    0, 0, 0, 0,
    //     -gyro[2]*DT,             1,    gyro[0]*DT,    0,    0,    0, 0, 0, 0,
    //      gyro[1]*DT,   -gyro[0]*DT,             1,    0,    0,    0, 0, 0, 0,
    //   -Cbissa[0]*DT, -Cbissa[1]*DT, -Cbissa[2]*DT,    1,    0,    0, 0, 0, 0,
    //   -Cbissa[3]*DT, -Cbissa[4]*DT, -Cbissa[5]*DT,    0,    1,    0, 0, 0, 0,
    //   -Cbissa[6]*DT, -Cbissa[7]*DT, -Cbissa[8]*DT,    0,    0,    1, 0, 0, 0,
    //               0,             0,             0, 1*DT,    0,    0, 1, 0, 0,
    //               0,             0,             0,    0, 1*DT,    0, 0, 1, 0,
    //               0,             0,             0,    0,    0, 1*DT, 0, 0, 1,
    // };

    const float Phi11[3*3] = {
                1,    gyro[2]*DT,   -gyro[1]*DT,
      -gyro[2]*DT,             1,    gyro[0]*DT,
       gyro[1]*DT,   -gyro[0]*DT,             1,
    };

    float Phi21[3*3];
    Vector3ToSkewSymmetric3(accelerometer, temp);
    MatrixMultiplySkewSymmetric3(Cbi, temp, 3, Phi21);
    MatrixScaleSelf(Phi21, -DT, 3, 3);  // Phi21 = Cbi * ssa * DT

    float PhiPPhit11[3*3], PhiPPhit12[3*3], PhiPPhit13[3*3];
    float PhiPPhit21[3*3], PhiPPhit22[3*3], PhiPPhit23[3*3];
    float PhiPPhit31[3*3], PhiPPhit32[3*3], PhiPPhit33[3*3];

    // PhiPPhit11
    MatrixMultiply(Phi11, P11, 3, 3, 3, temp);
    MatrixMultiplyByTranspose(temp, Phi11, 3, 3, 3, PhiPPhit11);

    // PhiPPhit12
    MatrixMultiplyByTranspose(temp, Phi21, 3, 3, 3, PhiPPhit12);
    MatrixMultiply(Phi11, P12, 3, 3, 3, temp);
    MatrixAddToSelf(PhiPPhit12, temp, 3, 3);

    // PhiPPhit13
    MatrixScaleSelf(temp, DT, 3, 3);
    MatrixMultiply(Phi11, P13, 3, 3, 3, PhiPPhit13);
    MatrixAddToSelf(PhiPPhit13, temp, 3, 3);

    // PhiPPhit21
    MatrixMultiply(Phi21, P11, 3, 3, 3, temp);
    MatrixAddToSelf(temp, P21, 3, 3);
    MatrixMultiplyByTranspose(temp, Phi11, 3, 3, 3, PhiPPhit21);

    // PhiPPhit22
    MatrixMultiplyByTranspose(temp, Phi21, 3, 3, 3, PhiPPhit22);
    MatrixMultiply(Phi21, P12, 3, 3, 3, temp);
    MatrixAddToSelf(temp, P22, 3, 3);
    MatrixAddToSelf(PhiPPhit22, temp, 3, 3);

    // PhiPPhit23
    MatrixScaleSelf(temp, DT, 3, 3);
    MatrixMultiply(Phi21, P13, 3, 3, 3, PhiPPhit23);
    MatrixAddToSelf(PhiPPhit23, P23, 3, 3);
    MatrixAddToSelf(PhiPPhit23, temp, 3, 3);

    // PhiPPhit31
    MatrixScale(P21, DT, 3, 3, temp);
    MatrixAddToSelf(temp, P31, 3, 3);
    MatrixMultiplyByTranspose(temp, Phi11, 3, 3, 3, PhiPPhit31);

    // PhiPPhit32
    MatrixMultiplyByTranspose(temp, Phi21, 3, 3, 3, PhiPPhit32);
    MatrixScale(P22, DT, 3, 3, temp);
    MatrixAddToSelf(temp, P32, 3, 3);
    MatrixAddToSelf(PhiPPhit32, temp, 3, 3);

    // PhiPPhit33
    MatrixScaleSelf(temp, DT, 3, 3);
    MatrixScale(P23, DT, 3, 3, PhiPPhit33);
    MatrixAddToSelf(PhiPPhit33, P33, 3, 3);
    MatrixAddToSelf(PhiPPhit33, temp, 3, 3);

    MatrixCopyToSubmatrix(PhiPPhit11, PhiPPhit, 0, 0, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit12, PhiPPhit, 0, 3, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit13, PhiPPhit, 0, 6, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit21, PhiPPhit, 3, 0, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit22, PhiPPhit, 3, 3, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit23, PhiPPhit, 3, 6, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit31, PhiPPhit, 6, 0, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit32, PhiPPhit, 6, 3, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit33, PhiPPhit, 6, 6, 3, 3, P_DIM);
  }

  // P_pred = Phi*P*Phi^t + Gamma*Q*Gamma^t + Qprime
  float GammaQGammat[P_DIM*P_DIM] = { 0 };
  {
    // Gamma = B * dt
    // const float Gamma[P_DIM * U_DIM] = {
    //   -1*DT,     0,     0,          0,          0,          0,
    //       0, -1*DT,     0,          0,          0,          0,
    //       0,     0, -1*DT,          0,          0,          0,
    //       0,     0,     0, -Cbi[0]*DT, -Cbi[1]*DT, -Cbi[2]*DT,
    //       0,     0,     0, -Cbi[3]*DT, -Cbi[4]*DT, -Cbi[5]*DT,
    //       0,     0,     0, -Cbi[6]*DT, -Cbi[7]*DT, -Cbi[8]*DT,
    //       0,     0,     0,          0,          0,          0,
    //       0,     0,     0,          0,          0,          0,
    //       0,     0,     0,          0,          0,          0,
    // };

    // const float QDiag[U_DIM] = {
    //   KALMAN_SIGMA_GYRO * KALMAN_SIGMA_GYRO,
    //   KALMAN_SIGMA_GYRO * KALMAN_SIGMA_GYRO,
    //   KALMAN_SIGMA_GYRO * KALMAN_SIGMA_GYRO,
    //   KALMAN_SIGMA_ACCELEROMETER_X * KALMAN_SIGMA_ACCELEROMETER_X,
    //   KALMAN_SIGMA_ACCELEROMETER_Y * KALMAN_SIGMA_ACCELEROMETER_Y,
    //   KALMAN_SIGMA_ACCELEROMETER_Z * KALMAN_SIGMA_ACCELEROMETER_Z,
    // };

    const float GammaQGammat11[3*3] = {
      -DT*KALMAN_SIGMA_GYRO*KALMAN_SIGMA_GYRO*-DT, 0, 0,
      0, -DT*KALMAN_SIGMA_GYRO*KALMAN_SIGMA_GYRO*-DT, 0,
      0, 0, -DT*KALMAN_SIGMA_GYRO*KALMAN_SIGMA_GYRO*-DT,
    };

    const float Gamma22[3*3] = {
      -Cbi[0]*DT, -Cbi[1]*DT, -Cbi[2]*DT,
      -Cbi[3]*DT, -Cbi[4]*DT, -Cbi[5]*DT,
      -Cbi[6]*DT, -Cbi[7]*DT, -Cbi[8]*DT,
    };

    float GammaQGammat22[3*3];
    const float Q22Gamma22t[3*3] = {
      -Cbi[0]*DT*KALMAN_SIGMA_ACCELEROMETER_X*KALMAN_SIGMA_ACCELEROMETER_X,
      -Cbi[3]*DT*KALMAN_SIGMA_ACCELEROMETER_X*KALMAN_SIGMA_ACCELEROMETER_X,
      -Cbi[6]*DT*KALMAN_SIGMA_ACCELEROMETER_X*KALMAN_SIGMA_ACCELEROMETER_X,
      -Cbi[1]*DT*KALMAN_SIGMA_ACCELEROMETER_Y*KALMAN_SIGMA_ACCELEROMETER_Y,
      -Cbi[4]*DT*KALMAN_SIGMA_ACCELEROMETER_Y*KALMAN_SIGMA_ACCELEROMETER_Y,
      -Cbi[7]*DT*KALMAN_SIGMA_ACCELEROMETER_Y*KALMAN_SIGMA_ACCELEROMETER_Y,
      -Cbi[2]*DT*KALMAN_SIGMA_ACCELEROMETER_Z*KALMAN_SIGMA_ACCELEROMETER_Z,
      -Cbi[5]*DT*KALMAN_SIGMA_ACCELEROMETER_Z*KALMAN_SIGMA_ACCELEROMETER_Z,
      -Cbi[8]*DT*KALMAN_SIGMA_ACCELEROMETER_Z*KALMAN_SIGMA_ACCELEROMETER_Z,
    };
    MatrixMultiply(Gamma22, Q22Gamma22t, 3, 3, 3, GammaQGammat22);

    MatrixCopyToSubmatrix(GammaQGammat11, GammaQGammat, 0, 0, 3, 3, 9);
    MatrixCopyToSubmatrix(GammaQGammat22, GammaQGammat, 3, 3, 3, 3, 9);
  }

  const float QprimeDiag[P_DIM] = { KALMAN_Q_PRIME_ALPHA * DT,
    KALMAN_Q_PRIME_ALPHA * DT, KALMAN_Q_PRIME_ALPHA * DT,
    KALMAN_Q_PRIME_VELOCITY * DT, KALMAN_Q_PRIME_VELOCITY * DT,
    KALMAN_Q_PRIME_VELOCITY * DT, KALMAN_Q_PRIME_H_POSITION * DT,
    KALMAN_Q_PRIME_H_POSITION * DT, KALMAN_Q_PRIME_V_POSITION * DT };

  // P_pred = Phi*P*Phi^t + Gamma*Q*Gamma^t + Qprime
  // recall that (float *)P_pred = (float *)PhiPPhit
  MatrixAddDiagonalToSelf(MatrixAddToSelf(PhiPPhit, GammaQGammat, P_DIM, P_DIM),
    QprimeDiag, P_DIM);
}

// -----------------------------------------------------------------------------
static void AccelerometerUpdate(const float * x_pred, const float * P_pred,
  const float * accelerometer, float * x_est, float * P_est)
{
  // There are 4 variables that are unique to each type of measurement.
  //
  // 1. z_dim: dimension of measurement vector z
  // 2. R_diag: array of diagonal elements of measurement covariance matrix R
  // 3. H: error state observation matrix, i.e. Jacobian of observation equation
  // 4. predicted_measurement: prediction of measurement vector z, calculated
  //    using prediction of state vector x (x_pred)
  //
  // After these variables become available, function MeasurementUpdateCommon
  // can calculate estimates (x_est and P_est) using prediction (x_pred and
  // P_pred), measurement (z), and the four variables.

  const float * quat_pred = &x_pred[0]; // predicted attitude quaternion

  // 2. Assign diagonal elements of R
  const float R_diag[2] = {
    KALMAN_SIGMA_ACCELEROMETER_G * KALMAN_SIGMA_ACCELEROMETER_G,
    KALMAN_SIGMA_ACCELEROMETER_G * KALMAN_SIGMA_ACCELEROMETER_G,
  };

  // 3. Assign elements of H
  // H = Cib * [[0 0 -G]^t x ]
  // const float H[2*P_DIM] = {
  //    G * C[1*3+0],  G * C[1*3+1],  G * C[1*3+2], 0, 0, 0, 0, 0, 0,
  //   -G * C[0*3+0], -G * C[0*3+1], -G * C[0*3+2], 0, 0, 0, 0, 0, 0,
  // };

  // Compute S = H*P*H^t + R.
  float C[3*3];
  QuaternionToDCM(quat_pred, C);  // C = DCM of current attitude
  const float H11[2*3] = {
     G * C[1*3+0],  G * C[1*3+1],  G * C[1*3+2],
    -G * C[0*3+0], -G * C[0*3+1], -G * C[0*3+2],
  };
  // const float H12[2*6] = { 0 };

  float P11[3*3], P12[3*6], P21[6*3], P22[6*6];
  SubmatrixCopyToMatrix(P_pred, P11, 0, 0, P_DIM, 3, 3);
  SubmatrixCopyToMatrix(P_pred, P12, 0, 3, P_DIM, 3, 6);
  SubmatrixCopyToMatrix(P_pred, P21, 3, 0, P_DIM, 6, 3);
  SubmatrixCopyToMatrix(P_pred, P22, 3, 3, P_DIM, 6, 6);

  float PHt[P_DIM*2];
  float * PHt11 = &PHt[0*2+0];  // 3x2
  float * PHt21 = &PHt[3*2+0];  // 6x2
  MatrixMultiplyByTranspose(P11, H11, 3, 3, 2, PHt11);
  MatrixMultiplyByTranspose(P21, H11, 6, 3, 2, PHt21);

  float HPHt11[2*2];
  MatrixMultiply(H11, PHt11, 2, 3, 2, HPHt11);

  float * S = HPHt11;  // Conserves memory
  MatrixAddDiagonalToSelf(S, R_diag, 2);

  float S_inv[2*2];
  // TODO: make a 2x2 specific inverse routine for speed.
  MatrixInverse(S, 2, S_inv);

  // Compute Kalman gain K = P*H^t*S^-1.
  float K[P_DIM*2];
  float * K11 = &K[0*2+0];  // 3x2
  float * K21 = &K[3*2+0];  // 6x2
  MatrixMultiply(PHt11, S_inv, 3, 2, 2, K11);
  MatrixMultiply(PHt21, S_inv, 6, 2, 2, K21);

  // Update error covariance matrix.
  // P_est = (I - KH) * P_pred
  {
    float IKH1[P_DIM*3];
    float * IKH11 = &IKH1[0*3+0];  // 3x3
    float * IKH21 = &IKH1[3*3+0];  // 6x3
    MatrixSubtractSelfFromIdentity(MatrixMultiply(K11, H11, 3, 2, 3, IKH11), 3);
    MatrixNegateSelf(MatrixMultiply(K21, H11, 6, 2, 3, IKH21), 6, 3);

    float P_est11[3*3], P_est12[3*6], P_est21[6*3], P_est22[6*6];
    MatrixMultiply(IKH11, P11, 3, 3, 3, P_est11);
    MatrixMultiply(IKH11, P12, 3, 3, 6, P_est12);
    MatrixAddToSelf(MatrixMultiply(IKH21, P11, 6, 3, 3, P_est21), P21, 6, 3);
    MatrixAddToSelf(MatrixMultiply(IKH21, P12, 6, 3, 6, P_est22), P22, 6, 6);

    MatrixCopyToSubmatrix(P_est11, P_est, 0, 0, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(P_est12, P_est, 0, 3, 3, 6, P_DIM);
    MatrixCopyToSubmatrix(P_est21, P_est, 3, 0, 6, 3, P_DIM);
    MatrixCopyToSubmatrix(P_est22, P_est, 3, 3, 6, 6, P_DIM);
  }

  // predicted measurement = DCM * a0
  float predicted_measurement[2] = { H11[1*3+2], -H11[0*3+2] };

  // Calculate innovation.
  float innovation[2];  // A.K.A. measurement residual
  VectorSubtract(accelerometer, predicted_measurement, 2, innovation);

  // delta = K * innovation
  float delta[P_DIM];
  MatrixMultiply(K, innovation, P_DIM, 2, 1, delta);

  // dq = Psi * alpha / 2
  float dq[4];
  {
    const float * quat_pred = &x_pred[0];
    float Psi[4*3];
    Psi[0*3+0] = -quat_pred[1];
    Psi[0*3+1] = -quat_pred[2];
    Psi[0*3+2] = -quat_pred[3];
    Psi[1*3+0] = quat_pred[0];
    Psi[1*3+1] = -quat_pred[3];
    Psi[1*3+2] = quat_pred[2];
    Psi[2*3+0] = quat_pred[3];
    Psi[2*3+1] = quat_pred[0];
    Psi[2*3+2] = -quat_pred[1];
    Psi[3*3+0] = -quat_pred[2];
    Psi[3*3+1] = quat_pred[1];
    Psi[3*3+2] = quat_pred[0];

    const float * alpha = &delta[0];  // alpha is the first 3 elements of delta
    VectorScaleSelf(MatrixMultiply(Psi, alpha, 4, 3, 1, dq), 0.5, 4);
  }

  x_est[0] = dq[0];
  x_est[1] = dq[1];
  x_est[2] = dq[2];
  x_est[3] = dq[3];
  x_est[4] = delta[3];
  x_est[5] = delta[4];
  x_est[6] = delta[5];
  x_est[7] = delta[6];
  x_est[8] = delta[7];
  x_est[9] = delta[8];
  VectorAddToSelf(x_est, x_pred, X_DIM);

  QuaternionNormalizingFilter(&x_est[0]);  // normalize the quaternion portion of x_est
}

// -----------------------------------------------------------------------------
static void BaroAltitudeUpdate(const float *x_pred, const float *P_pred,
  float baro_altitude, float *x_est, float *P_est)
{
  // There are 4 variables that are unique to each type of measurement.
  //
  // 1. z_dim: dimension of measurement vector z
  // 2. R_diag: array of diagonal elements of measurement covariance matrix R
  // 3. H: error state observation matrix, i.e. Jacobian of observation equation
  // 4. predicted_measurement: prediction of measurement vector z, calculated
  //    using prediction of state vector x (x_pred)
  //
  // After these variables become available, function MeasurementUpdateCommon
  // can calculate estimates (x_est and P_est) using prediction (x_pred and
  // P_pred), measurement (z), and the four variables.

  const float *r_pred = &x_pred[7]; // predicted position in i-frame

  // 2. Assign diagonal elements of R
  const float RDiag[1] = { KALMAN_SIGMA_BARO * KALMAN_SIGMA_BARO };

  // 3. Assign elements of H
  // measurement: barometric altitude = alt0 + (-1)*(rz + delta_rz)
  // predicted measurement: alt0 + (-1)*rz
  // innovation = measurement - predicted measurement
  //            = (-1)*delta_rz
  // Therefore H = [0 0 0  0 0 0  0 0 -1]
  // const float H[1*P_DIM] = { 0, 0, 0, 0, 0, 0, 0, 0, -1 };

  // Compute S = H*P*H^t + R.
  // const float H11[1*8] = { 0 };
  // const float H12[1*1] = { -1 };

  float P11[8*8], P12[8*1], P21[1*8], P22[1*1];
  SubmatrixCopyToMatrix(P_pred, P11, 0, 0, P_DIM, 8, 8);
  SubmatrixCopyToMatrix(P_pred, P12, 0, 8, P_DIM, 8, 1);
  SubmatrixCopyToMatrix(P_pred, P21, 8, 0, P_DIM, 1, 8);
  SubmatrixCopyToMatrix(P_pred, P22, 8, 8, P_DIM, 1, 1);

  float PHt[P_DIM*1];
  MatrixNegateSelf(SubmatrixCopyToMatrix(P_pred, PHt, 0, 8, P_DIM, 9, 1), 9, 1);

  // float HPHt[1*1];
  // HPHt[0*1+0] = P_pred[8*P_DIM+8];

  // float S_inv[1*1] = 1.0 / (HPHt[0*1+0] + RDiag[0*1+0]);
  float S_inv[1*1] = { 1.0 / (P22[0*1+0] + RDiag[0*1+0]) };

  // Compute Kalman gain K = P*H^t*S^-1.
  float * K = PHt;  // Conserves memory
  MatrixScaleSelf(K, S_inv[0*1+0], 9, 1);

  // const float H[1*P_DIM] = { 0, 0, 0, 0, 0, 0, 0, 0, -1 };
  // float predicted_measurement[1] = { -r_pred[2] + baro_altitude_offset };
  // MeasurementUpdateCommon(x_pred, P_pred, &baro_altitude, x_est, P_est, 1,
  //   RDiag, H, predicted_measurement);

  // Update error covariance matrix.
  // P_est = (I - KH) * P_pred
  {
    float IKH22[1*1] = { 1.0 + K[8*1+0] };

    float P_est11[8*8], P_est12[8*1], P_est21[1*8], P_est22[1*1];
    MatrixAddToSelf(MatrixMultiply(K, P21, 8, 1, 8, P_est11), P11, 8, 8);
    MatrixAddToSelf(MatrixMultiply(K, P22, 8, 1, 1, P_est12), P12, 8, 1);
    MatrixMultiply(IKH22, P21, 1, 1, 8, P_est21);
    MatrixMultiply(IKH22, P22, 1, 1, 1, P_est22);

    MatrixCopyToSubmatrix(P_est11, P_est, 0, 0, 8, 8, P_DIM);
    MatrixCopyToSubmatrix(P_est12, P_est, 0, 8, 8, 1, P_DIM);
    MatrixCopyToSubmatrix(P_est21, P_est, 8, 0, 1, 8, P_DIM);
    MatrixCopyToSubmatrix(P_est22, P_est, 8, 8, 1, 1, P_DIM);
  }

  // 4. Calculate predicted measurement
  float predicted_measurement[1] = { -r_pred[2] + baro_altitude_offset };

  // Calculate innovation.
  float innovation[1];  // A.K.A. measurement residual
  VectorSubtract(&baro_altitude, predicted_measurement, 1, innovation);

  // delta = K * innovation
  float delta[P_DIM];
  MatrixMultiply(K, innovation, P_DIM, 1, 1, delta);

  // dq = Psi * alpha / 2
  float dq[4];
  {
    const float * quat_pred = &x_pred[0];
    float Psi[4*3];
    Psi[0*3+0] = -quat_pred[1];
    Psi[0*3+1] = -quat_pred[2];
    Psi[0*3+2] = -quat_pred[3];
    Psi[1*3+0] = quat_pred[0];
    Psi[1*3+1] = -quat_pred[3];
    Psi[1*3+2] = quat_pred[2];
    Psi[2*3+0] = quat_pred[3];
    Psi[2*3+1] = quat_pred[0];
    Psi[2*3+2] = -quat_pred[1];
    Psi[3*3+0] = -quat_pred[2];
    Psi[3*3+1] = quat_pred[1];
    Psi[3*3+2] = quat_pred[0];

    const float * alpha = &delta[0];  // alpha is the first 3 elements of delta
    VectorScaleSelf(MatrixMultiply(Psi, alpha, 4, 3, 1, dq), 0.5, 4);
  }

  x_est[0] = dq[0];
  x_est[1] = dq[1];
  x_est[2] = dq[2];
  x_est[3] = dq[3];
  x_est[4] = delta[3];
  x_est[5] = delta[4];
  x_est[6] = delta[5];
  x_est[7] = delta[6];
  x_est[8] = delta[7];
  x_est[9] = delta[8];
  VectorAddToSelf(x_est, x_pred, X_DIM);

  QuaternionNormalizingFilter(&x_est[0]);  // normalize the quaternion portion of x_est
}

// -----------------------------------------------------------------------------
static void VisionUpdate(const float * x_pred, const float * P_pred,
  const float * vision, float * x_est, float * P_est)
{
  // There are 4 variables that are unique to each type of measurement.
  //
  // 1. z_dim: dimension of measurement vector z
  // 2. R_diag: array of diagonal elements of measurement covariance matrix R
  // 3. H: error state observation matrix, i.e. Jacobian of observation equation
  // 4. predicted_measurement: prediction of measurement vector z, calculated
  //    using prediction of state vector x (x_pred)
  //
  // After these variables become available, function MeasurementUpdateCommon
  // can calculate estimates (x_est and P_est) using prediction (x_pred and
  // P_pred), measurement (z), and the four variables.

  const float * quat_pred = &x_pred[0]; // predicted attitude quaternion
  const float * velocity_pred = &x_pred[4];  // predicted velocity in i-frame

  // 2. Assign diagonal elements of R
  const float R_diag[3] = {
    KALMAN_SIGMA_VISION * KALMAN_SIGMA_VISION,
    KALMAN_SIGMA_VISION * KALMAN_SIGMA_VISION,
    KALMAN_SIGMA_VISION * KALMAN_SIGMA_VISION,
  };

  // 3. Assign elements of H
  // H = Cib * [ C*ssv C 0 ]
  float C[3*3], H11[3*3], temp[3*3];
  QuaternionToDCM(quat_pred, C);  // C = DCM of current attitude
  float * ssv = temp;  // Conserves memory
  Vector3ToSkewSymmetric3(velocity_pred, ssv);  // ssv = skew-symmetric of v
  MatrixMultiply(C, ssv, 3, 3, 3, H11);
  const float * H12 = C;  // Conserves memory

  // Compute Kalman gain K = P*H^t*S^-1.
  float K[P_DIM*3];
  float P11[3*3], P12[3*3], P13[3*3];
  float P21[3*3], P22[3*3], P23[3*3];
  float P31[3*3], P32[3*3], P33[3*3];
  SubmatrixCopyToMatrix(P_pred, P11, 0, 0, P_DIM, 3, 3);
  SubmatrixCopyToMatrix(P_pred, P12, 0, 3, P_DIM, 3, 3);
  SubmatrixCopyToMatrix(P_pred, P13, 0, 6, P_DIM, 3, 3);
  SubmatrixCopyToMatrix(P_pred, P21, 3, 0, P_DIM, 3, 3);
  SubmatrixCopyToMatrix(P_pred, P22, 3, 3, P_DIM, 3, 3);
  SubmatrixCopyToMatrix(P_pred, P23, 3, 6, P_DIM, 3, 3);
  SubmatrixCopyToMatrix(P_pred, P31, 6, 0, P_DIM, 3, 3);
  SubmatrixCopyToMatrix(P_pred, P32, 6, 3, P_DIM, 3, 3);
  SubmatrixCopyToMatrix(P_pred, P33, 6, 6, P_DIM, 3, 3);
  {
    // Compute S = H*P*H^t + R
    float PHt[P_DIM*3];
    float * PHt11 = &PHt[0*3+0];  // 3x3
    float * PHt21 = &PHt[3*3+0];  // 3x3
    float * PHt31 = &PHt[6*3+0];  // 3x3
    MatrixMultiplyByTranspose(P11, H11, 3, 3, 3, temp);
    MatrixMultiplyByTranspose(P12, H12, 3, 3, 3, PHt11);
    MatrixAddToSelf(PHt11, temp, 3, 3);
    MatrixMultiplyByTranspose(P21, H11, 3, 3, 3, temp);
    MatrixMultiplyByTranspose(P22, H12, 3, 3, 3, PHt21);
    MatrixAddToSelf(PHt21, temp, 3, 3);
    MatrixMultiplyByTranspose(P31, H11, 3, 3, 3, temp);
    MatrixMultiplyByTranspose(P32, H12, 3, 3, 3, PHt31);
    MatrixAddToSelf(PHt31, temp, 3, 3);

    float HPHt[3*3];
    MatrixMultiply(H11, PHt11, 3, 3, 3, temp);
    MatrixMultiply(H12, PHt21, 3, 3, 3, HPHt);
    MatrixAddToSelf(HPHt, temp, 3, 3);

    float * S = HPHt;  // Conserves memory
    MatrixAddDiagonalToSelf(S, R_diag, 3);

    float S_inv[3*3];
    MatrixInverse(S, 3, S_inv);

    MatrixMultiply(PHt, S_inv, P_DIM, 3, 3, K);
  }

  // Update error covariance matrix.
  // P_est = (I - KH) * P_pred
  {
    float * K11 = &K[0*3+0];  // 3x3
    float * K21 = &K[3*3+0];  // 3x3
    float * K31 = &K[6*3+0];  // 3x3

    float IKH11[3*3], IKH12[3*3];
    float IKH21[3*3], IKH22[3*3];
    float IKH31[3*3], IKH32[3*3];
    MatrixSubtractSelfFromIdentity(MatrixMultiply(K11, H11, 3, 3, 3, IKH11), 3);
    MatrixNegateSelf(MatrixMultiply(K11, H12, 3, 3, 3, IKH12), 3, 3);
    MatrixNegateSelf(MatrixMultiply(K21, H11, 3, 3, 3, IKH21), 3, 3);
    MatrixSubtractSelfFromIdentity(MatrixMultiply(K21, H12, 3, 3, 3, IKH22), 3);
    MatrixNegateSelf(MatrixMultiply(K31, H11, 3, 3, 3, IKH31), 3, 3);
    MatrixNegateSelf(MatrixMultiply(K31, H12, 3, 3, 3, IKH32), 3, 3);

    float P_est11[3*3], P_est12[3*3], P_est13[3*3];
    float P_est21[3*3], P_est22[3*3], P_est23[3*3];
    float P_est31[3*3], P_est32[3*3], P_est33[3*3];

    MatrixMultiply(IKH11, P11, 3, 3, 3, temp);
    MatrixAddToSelf(MatrixMultiply(IKH12, P21, 3, 3, 3, P_est11), temp, 3, 3);
    MatrixMultiply(IKH11, P12, 3, 3, 3, temp);
    MatrixAddToSelf(MatrixMultiply(IKH12, P22, 3, 3, 3, P_est12), temp, 3, 3);
    MatrixMultiply(IKH11, P13, 3, 3, 3, temp);
    MatrixAddToSelf(MatrixMultiply(IKH12, P23, 3, 3, 3, P_est13), temp, 3, 3);
    MatrixMultiply(IKH21, P11, 3, 3, 3, temp);
    MatrixAddToSelf(MatrixMultiply(IKH22, P21, 3, 3, 3, P_est21), temp, 3, 3);
    MatrixMultiply(IKH21, P12, 3, 3, 3, temp);
    MatrixAddToSelf(MatrixMultiply(IKH22, P22, 3, 3, 3, P_est22), temp, 3, 3);
    MatrixMultiply(IKH21, P13, 3, 3, 3, temp);
    MatrixAddToSelf(MatrixMultiply(IKH22, P23, 3, 3, 3, P_est23), temp, 3, 3);
    MatrixAddToSelf(MatrixMultiply(IKH31, P11, 3, 3, 3, temp), P31, 3, 3);
    MatrixAddToSelf(MatrixMultiply(IKH32, P21, 3, 3, 3, P_est31), temp, 3, 3);
    MatrixAddToSelf(MatrixMultiply(IKH31, P12, 3, 3, 3, temp), P32, 3, 3);
    MatrixAddToSelf(MatrixMultiply(IKH32, P22, 3, 3, 3, P_est32), temp, 3, 3);
    MatrixAddToSelf(MatrixMultiply(IKH31, P13, 3, 3, 3, temp), P33, 3, 3);
    MatrixAddToSelf(MatrixMultiply(IKH32, P23, 3, 3, 3, P_est33), temp, 3, 3);

    MatrixCopyToSubmatrix(P_est11, P_est, 0, 0, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(P_est12, P_est, 0, 3, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(P_est13, P_est, 0, 6, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(P_est21, P_est, 3, 0, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(P_est22, P_est, 3, 3, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(P_est23, P_est, 3, 6, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(P_est31, P_est, 6, 0, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(P_est32, P_est, 6, 3, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(P_est33, P_est, 6, 6, 3, 3, P_DIM);
  }

  // // predicted measurement = DCM * a0
  float predicted_measurement[3];
  MatrixMultiply(C, velocity_pred, 3, 3, 1, predicted_measurement);

  // Calculate innovation.
  float innovation[3];  // A.K.A. measurement residual
  VectorSubtract(vision, predicted_measurement, 3, innovation);

  // delta = K * innovation
  float delta[P_DIM];
  MatrixMultiply(K, innovation, P_DIM, 3, 1, delta);

  // dq = Psi * alpha / 2
  float dq[4];
  {
    const float * quat_pred = &x_pred[0];
    float Psi[4*3];
    Psi[0*3+0] = -quat_pred[1];
    Psi[0*3+1] = -quat_pred[2];
    Psi[0*3+2] = -quat_pred[3];
    Psi[1*3+0] = quat_pred[0];
    Psi[1*3+1] = -quat_pred[3];
    Psi[1*3+2] = quat_pred[2];
    Psi[2*3+0] = quat_pred[3];
    Psi[2*3+1] = quat_pred[0];
    Psi[2*3+2] = -quat_pred[1];
    Psi[3*3+0] = -quat_pred[2];
    Psi[3*3+1] = quat_pred[1];
    Psi[3*3+2] = quat_pred[0];

    const float * alpha = &delta[0];  // alpha is the first 3 elements of delta
    VectorScaleSelf(MatrixMultiply(Psi, alpha, 4, 3, 1, dq), 0.5, 4);
  }

  x_est[0] = dq[0];
  x_est[1] = dq[1];
  x_est[2] = dq[2];
  x_est[3] = dq[3];
  x_est[4] = delta[3];
  x_est[5] = delta[4];
  x_est[6] = delta[5];
  x_est[7] = delta[6];
  x_est[8] = delta[7];
  x_est[9] = delta[8];
  VectorAddToSelf(x_est, x_pred, X_DIM);

  QuaternionNormalizingFilter(&x_est[0]);  // normalize the quaternion portion of x_est
}

// -----------------------------------------------------------------------------
static void MeasurementUpdateCommon(const float * x_pred, const float * P_pred,
  const float * z, float * x_est, float * P_est, int z_dim,
  const float * R_diag, const float * H, const float * predicted_measurement)
{
  // Calculate Kalman gain (this code is common for all measurements).
  // Compute S = H*P*H^t + R.
  float S[Z_DIM_MAX*Z_DIM_MAX];
  float PHt[P_DIM*Z_DIM_MAX];
  {
    float Ht[P_DIM*Z_DIM_MAX];
    MatrixTranspose(H, z_dim, P_DIM, Ht);  // Ht = H^t
//////////////////////////////////////////////////////////////////////////////// 243
    MatrixMultiply(P_pred, Ht, P_DIM, P_DIM, z_dim, PHt);  // PHt = P*H^t
//////////////////////////////////////////////////////////////////////////////// 81
    MatrixAddDiagonalToSelf(MatrixMultiply(H, PHt, z_dim, P_DIM, z_dim, S),
      R_diag, z_dim);  // S = H*P*H^t + R
  }

  // Compute Kalman gain K = P*H^t*S^-1.
  float K[P_DIM*Z_DIM_MAX];
  {
    float S_inv[Z_DIM_MAX*Z_DIM_MAX];
//////////////////////////////////////////////////////////////////////////////// 51 multiplications and 3 divisions
    MatrixInverse(S, z_dim, S_inv);  // S_inv = S^-1
//////////////////////////////////////////////////////////////////////////////// 81
    MatrixMultiply(PHt, S_inv, P_DIM, z_dim, z_dim, K);  // K = P*H^t*S^-1

  }

  // Update error covariance matrix (This code is common for all measurements.)
  // P_est = (I - KH) * P_pred => OUTPUT 1 of 2
  {
    float KH[P_DIM*P_DIM];
//////////////////////////////////////////////////////////////////////////////// 243
    MatrixMultiply(K, H, P_DIM, z_dim, P_DIM, KH); // KH = K*H
//////////////////////////////////////////////////////////////////////////////// 729
    MatrixMultiply(MatrixSubtractSelfFromIdentity(KH, P_DIM), P_pred, P_DIM,
      P_DIM, P_DIM, P_est);
  }

  // Update state vector (This code is common for all measurements).
  // delta = [alpha;dv;dr] is a 9-element vector, which is the direct output of
  //   measurement update and corresponds to P, a 9x9 error covariance matrix.
  // dx = [dq;dv;dr] is a 10-element vector and performs correction to x_pred,
  //   i.e. x_est = x_pred + dx.
  // Conversion from alpha to dq is done as follows.
  //
  // q_est = q_pred x { 1       } , where x denotes quaternion product.
  //                  { alpha/2 }
  // Under the assumption that alpha<<1, further calculation shows
  // q_est = q_pred + Psi/2 * alpha, where * denotes matrix product and
  //       [-q1 -q2 -q3]
  // Psi = [ q0 -q3  q2]
  //       [ q3  q0 -q1]
  //       [-q2  q1  q0]
  //
  // Therefore dq = Psi/2 * alpha.

  // Calculate innovation (this code is common for all measurements).
  float innovation[Z_DIM_MAX];  // A.K.A. measurement residual
  VectorSubtract(z, predicted_measurement, z_dim, innovation);

  // delta = K * innovation
  float delta[P_DIM];
//////////////////////////////////////////////////////////////////////////////// 21
  MatrixMultiply(K, innovation, P_DIM, z_dim, 1, delta);

  // dq = Psi * alpha / 2
  float dq[4];
  {
    const float * quat_pred = &x_pred[0];
    float Psi[4*3];
    Psi[0*3+0] = -quat_pred[1];
    Psi[0*3+1] = -quat_pred[2];
    Psi[0*3+2] = -quat_pred[3];
    Psi[1*3+0] = quat_pred[0];
    Psi[1*3+1] = -quat_pred[3];
    Psi[1*3+2] = quat_pred[2];
    Psi[2*3+0] = quat_pred[3];
    Psi[2*3+1] = quat_pred[0];
    Psi[2*3+2] = -quat_pred[1];
    Psi[3*3+0] = -quat_pred[2];
    Psi[3*3+1] = quat_pred[1];
    Psi[3*3+2] = quat_pred[0];

    const float * alpha = &delta[0];  // alpha is the first 3 elements of delta
//////////////////////////////////////////////////////////////////////////////// 12 + 4
    VectorScaleSelf(MatrixMultiply(Psi, alpha, 4, 3, 1, dq), 0.5, 4);
  }

  x_est[0] = dq[0];
  x_est[1] = dq[1];
  x_est[2] = dq[2];
  x_est[3] = dq[3];
  x_est[4] = delta[3];
  x_est[5] = delta[4];
  x_est[6] = delta[5];
  x_est[7] = delta[6];
  x_est[8] = delta[7];
  x_est[9] = delta[8];
  VectorAddToSelf(x_est, x_pred, X_DIM);

  QuaternionNormalizingFilter(&x_est[0]);  // normalize the quaternion portion of x_est
}

// -----------------------------------------------------------------------------
static float * MatrixMultiplySkewSymmetric3(const float * A, const float B[3*3],
  size_t A_rows, float * result)
{
  for (size_t i = 0; i < A_rows; i++)
  {
    for (size_t j = 0; j < 3; j++)
    {
      result[i * 3 + j] = 0;
      for (size_t k = 0; k < 3; k++)
      {
        if (j != k) result[i * 3 + j] += A[3 * i + k] * B[j + 3 * k];
      }
    }
  }

  return result;
}

// -----------------------------------------------------------------------------
static float * QuaternionToDCM(const float *quat, float * result)
{
  float temp;

  result[0*3+0] = quat[0] * quat[0];
  result[1*3+1] = quat[2] * quat[2];
  temp = quat[1] * quat[1];
  result[2*3+2] = 0.5 - temp - result[1*3+1];
  result[1*3+1] += result[0*3+0] - 0.5;
  result[0*3+0] += temp - 0.5;

  result[0*3+1] = quat[1] * quat[2];
  result[1*3+0] = result[0*3+1];
  temp = quat[0] * quat[3];
  result[1*3+0] -= temp;
  result[0*3+1] += temp;

  result[0*3+2] = quat[1] * quat[3];
  result[2*3+0] = result[0*3+2];
  temp = quat[0] * quat[2];
  result[2*3+0] += temp;
  result[0*3+2] -= temp;

  result[1*3+2] = quat[2] * quat[3];
  result[2*3+1] = result[1*3+2];
  temp = quat[0] * quat[1];
  result[2*3+1] -= temp;
  result[1*3+2] += temp;

  // Double the result.
  for (size_t i = 0; i < 3*3; i++) result[i] += result[i];

  return result;
}

// -----------------------------------------------------------------------------
// static float * SkewSymmetric3Transpose(const float A[3*3] , float result[3*3])
// {
//   result[0*3+0] = 0.0;
//   result[0*3+1] = -A[0*3+1];
//   result[0*3+2] = -A[0*3+2];

//   result[1*3+0] = -A[1*3+0];
//   result[1*3+1] = 0.0;
//   result[1*3+2] = -A[1*3+2];

//   result[2*3+0] = -A[2*3+0];
//   result[2*3+1] = -A[2*3+1];
//   result[2*3+2] = 0.0;

//   return result;
// }

// -----------------------------------------------------------------------------
static float * UpdateQuaternion(const float quat[4],
  const float angular_rate[3], float result[4])
{
  float dq[3];
  Vector3Scale(angular_rate, 0.5 * DT, dq);

  result[0] = -dq[0] * quat[1] - dq[1] * quat[2] - dq[2] * quat[3] + quat[0];
  result[1] =  dq[0] * quat[0] - dq[1] * quat[3] + dq[2] * quat[2] + quat[1];
  result[2] =  dq[0] * quat[3] + dq[1] * quat[0] - dq[2] * quat[1] + quat[2];
  result[3] = -dq[0] * quat[2] + dq[1] * quat[1] + dq[2] * quat[0] + quat[3];

  QuaternionNormalizingFilter(result);

  return result;
}

// -----------------------------------------------------------------------------
static float * Vector3ToSkewSymmetric3(const float v[3], float * result)
{
  result[0*3+0] = 0.0;
  result[0*3+1] = -v[2];
  result[0*3+2] = v[1];

  result[1*3+0] = v[2];
  result[1*3+1] = 0.0;
  result[1*3+2] = -v[0];

  result[2*3+0] = -v[1];
  result[2*3+1] = v[0];
  result[2*3+2] = 0.0;

  return result;
}
