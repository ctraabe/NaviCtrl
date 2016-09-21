#ifndef MAG_CALIBRATION_H_
#define MAG_CALIBRATION_H_


// =============================================================================
// Accessors:

const float * MagBiasVector(void);

// -----------------------------------------------------------------------------
const float * MagGainVector(void);


// =============================================================================
// Public functions:

void MagCalibrationInit(void);

// -----------------------------------------------------------------------------
void MagCalibrationAddSample(void);

// -----------------------------------------------------------------------------
void MagCalibrationCopmute(void);


#endif  // MAG_CALIBRATION_H_
