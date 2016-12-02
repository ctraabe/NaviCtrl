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
#define SIGMA_POSITION_TX1 (0.01)
#define SIGMA_VELOCITY_RICOH (0.01)

static float heading_ = 0.0, position_[3] = { 0.0 }, velocity_ned_[3] = { 0.0 },
  quaternion_[4] = { 1.0, 0.0, 0.0, 0.0 };
static uint16_t status_;
static uint32_t timestamp_ = 0, last_reception_timestamp_ = 0;
static enum VisionErrorBits vision_error_bits_ = VISION_ERROR_BIT_STALE;


// =============================================================================
// Private function declarations:

static void VelocityFromPosition(const float sigma_position[3]);
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
uint32_t VisionDataStale(void)
{
  return vision_error_bits_ & VISION_ERROR_BIT_STALE;
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

// -----------------------------------------------------------------------------
float * VisionVelocityNEDVector(void)
{
  return velocity_ned_;
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
    // Update vision data to FlightCtrl to reflect staleness.
    VisionUpdates();
  }
}

// -----------------------------------------------------------------------------
void ProcessRaspiVisionData(struct RaspiVision * from_raspi)
{
  // Copy received data.
  status_ = from_raspi->status;
  timestamp_ = from_raspi->timestamp;
  Vector3Copy(from_raspi->position, position_);
  heading_ = from_raspi->heading;

  // Compute NED velocity by differentiating the position.
  VelocityFromPosition(from_raspi->position_sigma);

  VisionUpdates();
}

// -----------------------------------------------------------------------------
void ProcessRicohVisionData(struct RicohVision * from_ricoh)
{
  status_ = from_ricoh->reliability;
  timestamp_ = from_ricoh->capture_time;

  // Convert position from mm to m.
  Vector3Copy(Vector3ScaleSelf(from_ricoh->position, 1.0 / 1000.0), position_);

  // Compute full quaternion.
  quaternion_[1] = from_ricoh->quaternion[0];
  quaternion_[2] = from_ricoh->quaternion[1];
  quaternion_[3] = from_ricoh->quaternion[2];
  quaternion_[0] = sqrt(1.0 - quaternion_[1] * quaternion_[1] - quaternion_[2]
    * quaternion_[2] - quaternion_[3] * quaternion_[3]);

  // Compute heading angle.
  heading_ = HeadingFromQuaternion(quaternion_);

  // Convert velocity from mm/frame (at 30 fps) to MED m/s.
  Vector3ScaleSelf(from_ricoh->velocity, 30.0 / 1000.0);
  QuaternionRotateVector(quaternion_, from_ricoh->velocity, velocity_ned_);

  if (status_ == 1)
  {
    float r_velocity_ned[3] = { SIGMA_VELOCITY_RICOH, SIGMA_VELOCITY_RICOH,
      SIGMA_VELOCITY_RICOH };
    KalmanVelocityMeasurementUpdate(velocity_ned_, r_velocity_ned);
  }

  VisionUpdates();
}

// -----------------------------------------------------------------------------
void ProcessTX1VisionData(struct TX1Vision * from_tx1)
{
  // Copy received data.
  status_ = from_tx1->status;
  timestamp_ = from_tx1->timestamp;
  Vector3Copy(from_tx1->position, position_);

  // Compute full quaternion.
  quaternion_[1] = from_tx1->quaternion[0];
  quaternion_[2] = from_tx1->quaternion[1];
  quaternion_[3] = from_tx1->quaternion[2];
  quaternion_[0] = sqrt(1.0 - quaternion_[1] * quaternion_[1] - quaternion_[2]
    * quaternion_[2] - quaternion_[3] * quaternion_[3]);

  // Compute heading.
  heading_ = HeadingFromQuaternion(quaternion_);

  // Compute NED velocity by differentiating the position.
  float sigma_position[3] = { SIGMA_POSITION_TX1, SIGMA_POSITION_TX1,
    SIGMA_POSITION_TX1 };
  VelocityFromPosition(sigma_position);

  VisionUpdates();
}


// =============================================================================
// Private functions:

static void VelocityFromPosition(const float sigma_position[3])
{
  static uint32_t timestamp_pv = 0;
  static float position_pv[3] = { 0.0 }, sigma_pv[3] = { 0.0 };
  if (status_ == 1)
  {
    uint32_t dt_us = timestamp_ - timestamp_pv;
    if (dt_us < 1000) dt_us = 1000;
    float dt_inv = 1e6 / (float)dt_us;

    // Differentiate the position.
    Vector3ScaleSelf(Vector3Subtract(position_, position_pv, velocity_ned_),
      dt_inv);

    // Estimate the error covariance.
    float sigma_velocity_ned[3];
    Vector3ScaleSelf(Vector3Add(sigma_position, sigma_pv,
      sigma_velocity_ned), 0.5 * sqrt(2) * dt_inv);

    // Update the Kalman filter.
    KalmanVelocityMeasurementUpdate(velocity_ned_, sigma_velocity_ned);

    // Update past values.
    timestamp_pv = timestamp_;
    Vector3Copy(position_, position_pv);
    Vector3Copy(sigma_position, sigma_pv);
  }
}

// -----------------------------------------------------------------------------
static void VisionUpdates(void)
{
  UpdatePositionToFlightCtrl();
  UpdateHeadingCorrectionToFlightCtrl();
#ifdef LOG_DEBUG_TO_SD
  // LogTX1VisionData(from_tx1);
  // LogKalmanData;
#endif
}
