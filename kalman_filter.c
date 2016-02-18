#include "kalman_filter.h"

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
#define KALMAN_Q_PRIME_ALPHA (0)
#define KALMAN_Q_PRIME_VELOCITY (0.001)
#define KALMAN_Q_PRIME_POSITION (0.1)
#define KALMAN_SIGMA_ACCELEROMETER_X (0.005)
#define KALMAN_SIGMA_ACCELEROMETER_Y (0.005)
#define KALMAN_SIGMA_ACCELEROMETER_Z (0.042)
#define KALMAN_SIGMA_GYRO (0.007)
#define KALMAN_SIGMA_VISION (0.02)

static float x_[X_DIM] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
static float P_[P_DIM*P_DIM] = {
  1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
};


// =============================================================================
// Accessors:

const float * KalmanPosition(void)
{
  return &x_[7];
}

// -----------------------------------------------------------------------------
const float * KalmanX(void)
{
  return x_;
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
static void VisionUpdate(float * x_pred, float * P_pred, const float * vision,
  float * x_est, float * P_est);
static void MeasurementUpdateCommon(const float * x_pred, const float * P_pred,
  const float * z, float * x_est, float * P_est, int z_dim,
  const float * R_diag, const float * H, const float * predicted_measurement);
static float * QuaternionToDCM(const float *quat, float *result);
static float * SkewSymmetricFromVector3(const float vec[3], float *result);
static float * UpdateQuaternion(const float quat[4],
  const float angular_rate[3], float result[4]);


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
  x_[0] = 1.0;
  for (size_t i = 10; --i; ) x_[i] = 0.0;
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

  // Form body to inertial direction-cosine matrix.
  float Cbi[3 * 3], temp[3*3];
  MatrixTranspose(QuaternionToDCM(quat_prev, temp), 3, 3, Cbi);

  // Transform acceleration measured in the body frame to the inertial frame.
  float acceleration[3];  // Specific force measured in the inertial frame
  MatrixMultiply(Cbi, accelerometer, 3, 3, 1, acceleration);
  acceleration[2] += GRAVITY_ACCELERATION;  // Remove accelerometer gravity bias

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

  // TODO: reduce the memory usage of the following A and B computations
  // 1. Calculate Phi and Gamma
  float ssw[3*3];  // skew-symmetric form of angular rate vector w in b-frame
  float ssa[3*3];  // skew-symmetric form of specific force vector a in b-frame
  float Cbissa[3*3];  // matrix product of Cbi and ssa
  SkewSymmetricFromVector3(gyro, ssw);
  SkewSymmetricFromVector3(accelerometer, ssa);
  MatrixMultiply(Cbi, ssa, 3, 3, 3, Cbissa);  // Cbissa = Cbi * ssa

  float A[P_DIM*P_DIM] = {
       -ssw[0],    -ssw[1],    -ssw[2], 0, 0, 0, 0, 0, 0,
       -ssw[3],    -ssw[4],    -ssw[5], 0, 0, 0, 0, 0, 0,
       -ssw[6],    -ssw[7],    -ssw[8], 0, 0, 0, 0, 0, 0,
    -Cbissa[0], -Cbissa[1], -Cbissa[2], 0, 0, 0, 0, 0, 0,
    -Cbissa[3], -Cbissa[4], -Cbissa[5], 0, 0, 0, 0, 0, 0,
    -Cbissa[6], -Cbissa[7], -Cbissa[8], 0, 0, 0, 0, 0, 0,
             0,          0,          0, 1, 0, 0, 0, 0, 0,
             0,          0,          0, 0, 1, 0, 0, 0, 0,
             0,          0,          0, 0, 0, 1, 0, 0, 0,
  };
  float B[P_DIM * U_DIM] = {
    -1,  0,  0,       0,       0,       0,
     0, -1,  0,       0,       0,       0,
     0,  0, -1,       0,       0,       0,
     0,  0,  0, -Cbi[0], -Cbi[1], -Cbi[2],
     0,  0,  0, -Cbi[3], -Cbi[4], -Cbi[5],
     0,  0,  0, -Cbi[6], -Cbi[7], -Cbi[8],
     0,  0,  0,       0,       0,       0,
     0,  0,  0,       0,       0,       0,
     0,  0,  0,       0,       0,       0,
  };

  // Phi = I + A * dt
  float * Phi = A;  // conserves memory
  MatrixScaleSelf(A, DT, P_DIM, P_DIM);
  MatrixAddIdentityToSelf(Phi, P_DIM);

  // Gamma = B * dt
  float * Gamma = B;  // conserves memory
  MatrixScaleSelf(Gamma, DT, P_DIM, U_DIM);

  // 2. Define Q and Qprime
  const float QDiag[U_DIM] = { KALMAN_SIGMA_GYRO * KALMAN_SIGMA_GYRO,
    KALMAN_SIGMA_GYRO * KALMAN_SIGMA_GYRO,
    KALMAN_SIGMA_GYRO * KALMAN_SIGMA_GYRO,
    KALMAN_SIGMA_ACCELEROMETER_X * KALMAN_SIGMA_ACCELEROMETER_X,
    KALMAN_SIGMA_ACCELEROMETER_Y * KALMAN_SIGMA_ACCELEROMETER_Y,
    KALMAN_SIGMA_ACCELEROMETER_Z * KALMAN_SIGMA_ACCELEROMETER_Z };
  const float QprimeDiag[P_DIM] = { KALMAN_Q_PRIME_ALPHA * DT,
    KALMAN_Q_PRIME_ALPHA * DT, KALMAN_Q_PRIME_ALPHA * DT,
    KALMAN_Q_PRIME_VELOCITY * DT, KALMAN_Q_PRIME_VELOCITY * DT,
    KALMAN_Q_PRIME_VELOCITY * DT, KALMAN_Q_PRIME_POSITION * DT,
    KALMAN_Q_PRIME_POSITION * DT, KALMAN_Q_PRIME_POSITION * DT };

  // 3. Propagate P
  float * PhiPPhit = P_pred;  // conserves memory
  {
    float PhiP[P_DIM*P_DIM];
    float Phit[P_DIM*P_DIM];
    // PhiPPhit = Phi*P*Phi^t
    MatrixMultiply(Phi, P_est_prev, P_DIM, P_DIM, P_DIM, PhiP);
    MatrixTranspose(Phi, P_DIM, P_DIM, Phit);
    MatrixMultiply(PhiP, Phit, P_DIM, P_DIM, P_DIM, PhiPPhit);
  }

  // TODO: the following is very sparse and can be optimized
  float GammaQGammat[P_DIM*P_DIM];
  {
    // GammaQGammat = Gamma*Q*Gamma^t
    float Gammat[U_DIM*P_DIM];
    MatrixTranspose(Gamma, P_DIM, U_DIM, Gammat); // Gammat = Gamma^t
    MatrixMultiply(MatrixMultiplySelfByDiagonal(Gamma, QDiag, P_DIM, U_DIM),
      Gammat, P_DIM, U_DIM, P_DIM, GammaQGammat);
  }

  // P_pred = Phi*P*Phi^t + Gamma*Q*Gamma^t + Qprime => OUTPUT 2 of 2
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

  // a0 (specific force in initial orientation) is defined as the average of the
  // first several specific force measurements.
  const float a0[3] = { 0, 0, -GRAVITY_ACCELERATION };

  const float * quat_pred = &x_pred[0]; // predicted attitude quaternion

  // 2. Assign diagonal elements of R
  float R_diag[3];
  R_diag[0] = KALMAN_SIGMA_ACCELEROMETER_X * KALMAN_SIGMA_ACCELEROMETER_X;
  R_diag[1] = KALMAN_SIGMA_ACCELEROMETER_Y * KALMAN_SIGMA_ACCELEROMETER_Y;
  R_diag[2] = KALMAN_SIGMA_ACCELEROMETER_Z * KALMAN_SIGMA_ACCELEROMETER_Z;

  // 3. Assign elements of H
  float C[3*3];
  float Ca0[3*1];
  float ssCa0[3*3];
  float H[3*P_DIM] = { 0 };  // error state observation matrix

  // TODO: The following is very sparse and can be optimized.
  QuaternionToDCM(quat_pred, C);  // C = DCM of current attitude
  MatrixMultiply(C, a0, 3, 3, 1, Ca0);  // Ca0 = C*a0
  SkewSymmetricFromVector3(Ca0, ssCa0);  // ssCa0 = [(C*a0) x]

  H[0*P_DIM+0] = ssCa0[0*3+0];
  H[0*P_DIM+1] = ssCa0[0*3+1];
  H[0*P_DIM+2] = ssCa0[0*3+2];
  H[1*P_DIM+0] = ssCa0[1*3+0];
  H[1*P_DIM+1] = ssCa0[1*3+1];
  H[1*P_DIM+2] = ssCa0[1*3+2];
  H[2*P_DIM+0] = ssCa0[2*3+0];
  H[2*P_DIM+1] = ssCa0[2*3+1];
  H[2*P_DIM+2] = ssCa0[2*3+2];

  // 4. Calculate predicted measurement
  // predicted measurement = DCM * a0
  float predicted_measurement[3];
  MatrixMultiply(C, a0, 3, 3, 1, predicted_measurement);

  MeasurementUpdateCommon(x_pred, P_pred, accelerometer, x_est, P_est, 3,
    R_diag, H, predicted_measurement);
}

// -----------------------------------------------------------------------------
static void VisionUpdate(float * x_pred, float * P_pred, const float * vision,
  float * x_est, float * P_est)
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

  float * quat_pred = &x_pred[0];  // predicted attitude quaternion
  float * velocity_pred = &x_pred[4];  // predicted velocity in i-frame

  // 2. Assign diagonal elements of R
  float RDiag[3];
  RDiag[0] = KALMAN_SIGMA_VISION * KALMAN_SIGMA_VISION;
  RDiag[1] = KALMAN_SIGMA_VISION * KALMAN_SIGMA_VISION;
  RDiag[2] = KALMAN_SIGMA_VISION * KALMAN_SIGMA_VISION;

  // 3. Assign elements of H
  float C[3*3];
  float Cv[3];
  float ssCv[3*3];
  float H[3*P_DIM] = { 0 };  // error state observation matrix

  // TODO: The following is very sparse and can be optimized.
  QuaternionToDCM(quat_pred, C);  // C = DCM of current attitude
  MatrixMultiply(C, velocity_pred, 3, 3, 1, Cv);  // Cv = C*v
  SkewSymmetricFromVector3(Cv, ssCv);  // ssCv = skew-symmetric form of Cv

  H[0 * P_DIM + 0] = ssCv[0 * 3 + 0];
  H[0 * P_DIM + 1] = ssCv[0 * 3 + 1];
  H[0 * P_DIM + 2] = ssCv[0 * 3 + 2];
  H[1 * P_DIM + 0] = ssCv[1 * 3 + 0];
  H[1 * P_DIM + 1] = ssCv[1 * 3 + 1];
  H[1 * P_DIM + 2] = ssCv[1 * 3 + 2];
  H[2 * P_DIM + 0] = ssCv[2 * 3 + 0];
  H[2 * P_DIM + 1] = ssCv[2 * 3 + 1];
  H[2 * P_DIM + 2] = ssCv[2 * 3 + 2];

  H[0 * P_DIM + 3] = C[0 * 3 + 0];
  H[0 * P_DIM + 4] = C[0 * 3 + 1];
  H[0 * P_DIM + 5] = C[0 * 3 + 2];
  H[1 * P_DIM + 3] = C[1 * 3 + 0];
  H[1 * P_DIM + 4] = C[1 * 3 + 1];
  H[1 * P_DIM + 5] = C[1 * 3 + 2];
  H[2 * P_DIM + 3] = C[2 * 3 + 0];
  H[2 * P_DIM + 4] = C[2 * 3 + 1];
  H[2 * P_DIM + 5] = C[2 * 3 + 2];

  // 4. Calculate predicted measurement
  float predicted_measurement[3];
  MatrixMultiply(C, velocity_pred, 3, 3, 1, predicted_measurement);

  MeasurementUpdateCommon(x_pred, P_pred, vision, x_est, P_est, 3, RDiag, H,
    predicted_measurement);
}

// -----------------------------------------------------------------------------
static void MeasurementUpdateCommon(const float * x_pred, const float * P_pred,
  const float * z, float * x_est, float * P_est, int z_dim,
  const float * R_diag, const float * H, const float * predicted_measurement)
{
  // Calculate Kalman gain (this code is common for all measurements).
  // Compute S = H*P*H^t + R.
  float S[Z_DIM_MAX*Z_DIM_MAX];
  float Ht[P_DIM*Z_DIM_MAX];
  {
    float HP[Z_DIM_MAX*P_DIM];
    MatrixMultiply(H, P_pred, z_dim, P_DIM, P_DIM, HP);  // HP = H*P
    MatrixTranspose(H, z_dim, P_DIM, Ht);  // Ht = H^t
    MatrixAddDiagonalToSelf(MatrixMultiply(HP, Ht, z_dim, P_DIM, z_dim, S),
      R_diag, z_dim);
  }
  // Compute Kalman gain K = P*H^t*S^-1.
  float K[P_DIM*Z_DIM_MAX];
  {
    float PHt[P_DIM*Z_DIM_MAX];
    float S_inv[Z_DIM_MAX*Z_DIM_MAX];
    MatrixMultiply(P_pred, Ht, P_DIM, P_DIM, z_dim, PHt);  // PHt = P*H^t
    MatrixInverse(S, z_dim, S_inv);  // S_inv = S^-1
    MatrixMultiply(PHt, S_inv, P_DIM, z_dim, z_dim, K);
  }

  // Update error covariance matrix (This code is common for all measurements.)
  // P_est = (I - KH) * P_pred => OUTPUT 1 of 2
  {
    float KH[P_DIM*P_DIM];
    MatrixMultiply(K, H, P_DIM, z_dim, P_DIM, KH); // KH = K*H
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

  QuaternionNormalize(&x_est[0]);  // normalize the quaternion portion of x_est
}

// -----------------------------------------------------------------------------
static float * QuaternionToDCM(const float *quat, float *result)
{
  result[0*3+0] = quat[0] * quat[0] + quat[1] * quat[1] - quat[2] * quat[2]
    - quat[3] * quat[3];
  result[0*3+1] = 2 * (quat[1] * quat[2] + quat[0] * quat[3]);
  result[0*3+2] = 2 * (quat[1] * quat[3] - quat[0] * quat[2]);

  result[1*3+0] = 2 * (quat[1] * quat[2] - quat[0] * quat[3]);
  result[1*3+1] = quat[0] * quat[0] - quat[1] * quat[1] + quat[2] * quat[2]
    - quat[3] * quat[3];
  result[1*3+2] = 2 * (quat[2] * quat[3] + quat[0] * quat[1]);

  result[2*3+0] = 2 * (quat[1] * quat[3] + quat[0] * quat[2]);
  result[2*3+1] = 2 * (quat[2] * quat[3] - quat[0] * quat[1]);
  result[2*3+2] = quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2]
    + quat[3] * quat[3];

  return result;
}

// -----------------------------------------------------------------------------
static float * SkewSymmetricFromVector3(const float v[3], float *result)
{
  result[0*3+0] = 0;
  result[0*3+1] = -v[2];
  result[0*3+2] = v[1];

  result[1*3+0] = v[2];
  result[1*3+1] = 0;
  result[1*3+2] = -v[0];

  result[2*3+0] = -v[1];
  result[2*3+1] = v[0];
  result[2*3+2] = 0;

  return result;
}

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

  QuaternionNormalize(result);

  return result;
}
