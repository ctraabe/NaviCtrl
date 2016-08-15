#ifndef MAG_CALIBRATION_H_
#define MAG_CALIBRATION_H_


#include <stddef.h>


// =============================================================================
// Public functions:


void MagCalibrationInit(void);

// -----------------------------------------------------------------------------
void MagCalibrationAddSample(void);

// -----------------------------------------------------------------------------
void MagCalibratinCopmute(void);


#endif  // MAG_CALIBRATION_H_
