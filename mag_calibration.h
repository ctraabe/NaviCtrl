#ifndef MAG_CALIBRATION_H_
#define MAG_CALIBRATION_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

void MagCalibrationInit(const int16_t * const new_sample);

//------------------------------------------------------------------------------
uint32_t MagCalibration(uint32_t mag_calibration);


#endif  // MAG_CALIBRATION_H_
