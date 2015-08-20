#ifndef LSM303DL_H_
#define LSM303DL_H_


#include <inttypes.h>



// =============================================================================
// Accessors

int16_t * Magnetometer(void);


// =============================================================================
// Public functions:

void LSM303DLInit(void);

// -----------------------------------------------------------------------------
void LSM303DLReadMag(void);


#endif  // LSM303DL_H_
