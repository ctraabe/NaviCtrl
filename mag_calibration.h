#ifndef MAG_CALIBRATION_H_
#define MAG_CALIBRATION_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

void MagCalibrationInit(const int16_t * const new_sample);

// -----------------------------------------------------------------------------
void MagCalibrationAddSample(const int16_t * const new_sample);

// -----------------------------------------------------------------------------
void MagCalibrationCompute(float unitizer[3], int16_t bias[3]);


#endif  // MAG_CALIBRATION_H_
