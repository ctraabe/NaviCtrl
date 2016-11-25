#include "vision.h"

#include <math.h>
#include <stddef.h>

#include "attitude.h"
#include "flight_ctrl_comms.h"
#include "kalman_filter.h"
#include "quaternion.h"
#include "timing.h"
#include "vector.h"
#ifdef LOG_DEBUG_TO_SD
  #include logging.h
  #include main.h
#endif


// =============================================================================
// Private data:

#define VISION_FRESHNESS_LIMIT (100)  // millisends

static float heading_ = 0.0, position_[3] = { 0.0 },
  quaternion_[4] = { 1.0, 0.0, 0.0, 0.0 };
static uint16_t status_;
static uint32_t timestamp_ = 0, last_reception_timestamp_ = 0;
static enum VisionErrorBits vision_error_bits_ = VISION_ERROR_BIT_STALE;


// =============================================================================
// Private function declarations:

static void VisionUpdates(void);


// =============================================================================
// Accessors:

float VisionHeading(void)
{
  return heading_;
}

// -----------------------------------------------------------------------------
float VisionPosition(enum WorldAxes axis)
{
  return position_[axis];
}

// -----------------------------------------------------------------------------
const float * VisionPositionVector(void)
{
  return &position_[0];
}

// -----------------------------------------------------------------------------
const float * VisionQuaternionVector(void)
{
  return &quaternion_[0];
}

// -----------------------------------------------------------------------------
uint16_t VisionStatus(void)
{
  return status_;
}

// -----------------------------------------------------------------------------
uint32_t VisionTimestamp(void)
{
  return timestamp_;
}


// =============================================================================
// Public functions:

void CheckVisionFreshness(void)
{
  // Only check freshness if the data is not yet stale because the timestamp
  // might rollover, giving a false freshness.
  if ((~vision_error_bits_ & VISION_ERROR_BIT_STALE) &&
    (MillisSinceTimestamp(last_reception_timestamp_) > VISION_FRESHNESS_LIMIT))
  {
    vision_error_bits_ |= VISION_ERROR_BIT_STALE;
    status_ = 0;
  }
}

// -----------------------------------------------------------------------------
void ProcessRaspiVisionData(struct RaspiVision * from_raspi)
{
  // Copy received data.
  Vector3Copy(from_raspi->position, position_);
  heading_ = from_raspi->heading;
  status_ = from_raspi->status;
  timestamp_ = from_raspi->timestamp;

  VisionUpdates();
}

// -----------------------------------------------------------------------------
void ProcessRicohVisionData(struct RicohVision * from_ricoh)
{
  status_ = from_ricoh->reliability;
  // Convert angular rate from rad/frame (at 30 fps) to rad/s.
  // from_ricoh->angular_velocity[0] *= 30.0;
  // from_ricoh->angular_velocity[1] *= 30.0;
  // from_ricoh->angular_velocity[2] *= 30.0;

  // Convert position from mm to m.
  Vector3Copy(Vector3ScaleSelf(from_ricoh->position, 1.0 / 1000.0), position_);

  // Compute full quaternion.
  quaternion_[1] = from_ricoh->quaternion[0];
  quaternion_[2] = from_ricoh->quaternion[1];
  quaternion_[3] = from_ricoh->quaternion[2];
  quaternion_[0] = sqrt(1.0 - quaternion_[1] * quaternion_[1] - quaternion_[2]
    * quaternion_[2] - quaternion_[3] * quaternion_[3]);

  // Convert velocity from mm/frame (at 30 fps) to m/s.
  Vector3ScaleSelf(from_ricoh->velocity, 30.0 / 1000.0);

  // Compute inertial velocity.
  float inertial_velocity[3];
  QuaternionRotateVector(quaternion_, from_ricoh->velocity, inertial_velocity);
  KalmanVisionUpdateVelocity(inertial_velocity);

  // Compute heading angle.
  heading_ = HeadingFromQuaternion(quaternion_);

  VisionUpdates();
}

// -----------------------------------------------------------------------------
void ProcessTX1VisionData(struct TX1Vision * from_tx1)
{
  // Copy received data.
  Vector3Copy(from_tx1->position, position_);
  status_ = from_tx1->status;
  timestamp_ = from_tx1->timestamp;

  // Compute full quaternion.
  quaternion_[1] = from_tx1->quaternion[0];
  quaternion_[2] = from_tx1->quaternion[1];
  quaternion_[3] = from_tx1->quaternion[2];
  quaternion_[0] = sqrt(1.0 - quaternion_[1] * quaternion_[1] - quaternion_[2]
    * quaternion_[2] - quaternion_[3] * quaternion_[3]);

  // Compute heading.
  heading_ = HeadingFromQuaternion(quaternion_);

  VisionUpdates();
}


// =============================================================================
// Private functions:

static void VisionUpdates(void)
{
  UpdatePositionToFlightCtrl();
  UpdateHeadingCorrectionToFlightCtrl();
  KalmanVisionUpdateFromPosition();
#ifdef LOG_DEBUG_TO_SD
  // LogTX1VisionData(from_tx1);
  // LogKalmanData;
#endif
}
