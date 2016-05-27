#include "kalman_filter.h"

#include "attitude.h"
#include "constants.h"
#include "flight_ctrl_comms.h"
#include "quaternion.h"
#include "vector.h"
#include "vision.h"


// =============================================================================
// Private data:

#define Q_ACC_INTEGRAL (0.001)
#define R_VISION_DERIVATIVE (0.025)

static float p_ = 0.0;
static float velocity_[3] = { 0.0 };  // m/s


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

void KalmanTimeUpdate(void)
{
  float acceleration_ned[3];  // NED inertial axis
  QuaternionInverseRotateVector((float *)Quat(), (float *)AccelerometerVector(),
    acceleration_ned);

  // Remove acceleration due to gravity.
  acceleration_ned[D_WORLD_AXIS] -= -1.0;

  // Convert g's to m/s^2 and multiply by DT.
  Vector3ScaleSelf(acceleration_ned, GRAVITY_ACCELERATION * DT);

  // Integrate acceleration.
  Vector3AddToSelf(velocity_, acceleration_ned);

  // Update estimate error covariance.
  p_ += Q_ACC_INTEGRAL;
}

// -----------------------------------------------------------------------------
void KalmanVisionUpdate(void)
{
  static float dt = 1e6, position_pv[3] = { 0.0 };
  dt += (VisionDT() > 0.0) ? VisionDT() : 1e-2;
  float k = p_ / (p_ + R_VISION_DERIVATIVE);

  if (VisionStatus() == 1)
  {
    Vector3AddToSelf(Vector3ScaleSelf(velocity_, 1.0 - k),
      Vector3ScaleSelf(Vector3SubtractSelfFrom(position_pv,
      VisionPositionVector()), k / dt));

    // Update estimate error covariance, dt, and past value.
    p_ = (1.0 - k) * p_;
    dt = 0.0;
    Vector3Copy(VisionPositionVector(), position_pv);
  }
}
