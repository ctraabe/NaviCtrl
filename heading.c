#include "heading.h"

#include <math.h>

#include "flight_ctrl_comms.h"
#include "lsm303dl.h"
#include "quaternion.h"
#include "vector.h"


// =============================================================================
// Private data:

static float heading_angle_ = 0.0;
static float heading_correction_quat_0_ = 1.0, heading_correction_quat_z_ = 0.0;


// =============================================================================
// Accessors:

float HeadingAngle(void)
{
  return heading_angle_;
}

// -----------------------------------------------------------------------------
float HeadingCorrectionQuat0(void)
{
  return heading_correction_quat_0_;
}

// -----------------------------------------------------------------------------
float HeadingCorrectionQuatZ(void)
{
  return heading_correction_quat_z_;
}


// =============================================================================
// Public functions:

void UpdateHeading(void)
{
  heading_angle_ = atan2(2.0 * Quat()[0] * Quat()[3] + Quat()[1] * Quat()[2],
    1.0 - 2.0 * (Quat()[2] * Quat()[2] + Quat()[3] * Quat()[3]));

  // NOTE: currently declination is hard-coded to the value found in Tokyo!
  // TODO: Make declination and inclination a function of location
  const float declination = -7.0 * M_PI / 180.0;
  // const float inclination = -49.0;

  // Transform magnetometer reading back to earth axis.
  float magnetic_vector_earth[3];

  // Copy volatile quaternion to temporary storage.
  float temp[4] = {Quat()[0], Quat()[1], Quat()[2], Quat()[3]};
  QuaternionRotateVector(temp, MagneticVector(), magnetic_vector_earth);

  // TODO: Use measured inclination as a verification metric.

  // Remove declination.
  float dedeclinated_magnetic_vector[2] = { magnetic_vector_earth[0]
    * cos(declination) + magnetic_vector_earth[0] * sin(declination),
    magnetic_vector_earth[0] * -sin(declination) + magnetic_vector_earth[1]
    * cos(declination) };

  // The de-declinated magnetic vector should be along the North-Down plane, so
  // deviation of the measured value must be a result of attitude estimate
  // error, measurement noise, or an anomalous magnetic field. We assume that
  // the mean deviation is due to attitude estimate error.

  // Form a correction assuming small angle and discarding vertical.
  const float k_correction = 0.01;  // Must be less than 1/pi
  heading_correction_quat_z_ = -k_correction
    * atan2(dedeclinated_magnetic_vector[1], dedeclinated_magnetic_vector[0]);
  heading_correction_quat_0_ = sqrt(1 - heading_correction_quat_z_
    * heading_correction_quat_z_);
}
