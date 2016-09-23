#ifndef MAG_CALIBRATION_H_
#define MAG_CALIBRATION_H_


#include <inttypes.h>


// =============================================================================
// Accessors:

const float * MagBiasVector(void);

// -----------------------------------------------------------------------------
const float * MagGainVector(void);


// =============================================================================
// Public functions:

void MagCalibrationInit(const int16_t * const new_sample);

// -----------------------------------------------------------------------------
void MagCalibrationAddSample(const int16_t * const new_sample);

// -----------------------------------------------------------------------------
void MagCalibrationCopmute(void);


#endif  // MAG_CALIBRATION_H_
