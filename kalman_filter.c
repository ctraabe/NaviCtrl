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
#define R_VISION_VELOCITY (0.01)
#define R_VISION_POSITION_DERIVATIVE (0.025)

static float p_ = 1e6;
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

  UpdateVelocityToFlightCtrl();
}

// -----------------------------------------------------------------------------
void KalmanVisionUpdateVelocity(const float velocity[3])
{
  float k = p_ / (p_ + R_VISION_VELOCITY);

  if (VisionStatus() == 1)
  {
    float temp[3];
    Vector3AddToSelf(Vector3ScaleSelf(velocity_, 1.0 - k),
      Vector3Scale(velocity, k, temp));

    // Update estimate error covariance, last_timestamp, and past value.
    p_ = (1.0 - k) * p_;
    // TODO: remove this artificial limit
    if (p_ > 1e6) p_ = 1e6;

    UpdateVelocityToFlightCtrl();
  }
}

// -----------------------------------------------------------------------------
void KalmanVisionUpdateFromPosition(void)
{
  static uint32_t last_timestamp = 0;
  static float position_pv[3] = { 0.0 };
  float k = p_ / (p_ + R_VISION_POSITION_DERIVATIVE);

  if (VisionStatus() == 1)
  {
    uint32_t timestamp = VisionTimestamp();
    float dt = (float)(timestamp - last_timestamp) * 1e-6;
    if (dt < 1e-2) dt = 1e-2;

    Vector3AddToSelf(Vector3ScaleSelf(velocity_, 1.0 - k),
      Vector3ScaleSelf(Vector3SubtractSelfFrom(position_pv,
      VisionPositionVector()), k / dt));

    // Update estimate error covariance, last_timestamp, and past value.
    p_ = (1.0 - k) * p_;
    last_timestamp = timestamp;
    Vector3Copy(VisionPositionVector(), position_pv);

    UpdateVelocityToFlightCtrl();
  }
}
