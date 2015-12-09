#ifndef LSM303DL_H_
#define LSM303DL_H_


#include <inttypes.h>


// =============================================================================
// Accessors:

const int16_t * MagnetometerVector(void);

// -----------------------------------------------------------------------------
const float * MagneticVector(void);


// =============================================================================
// Public functions:

void LSM303DLInit(void);

// -----------------------------------------------------------------------------
void LSM303DLReadMag(void);


#endif  // LSM303DL_H_
