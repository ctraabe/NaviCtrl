#ifndef LSM303DL_H_
#define LSM303DL_H_


#include <inttypes.h>


enum LSM303DLErrorBits {
  LSM303DL_ERROR_BIT_NOT_INITIALIZED = 1<<0,
  LSM303DL_ERROR_BIT_I2C_BUSY        = 1<<1,
  LSM303DL_ERROR_BIT_DATA_WAITING    = 1<<2,
};


// =============================================================================
// Accessors:


uint32_t LSM303DLDataWaiting(void);

// -----------------------------------------------------------------------------
uint32_t LSM303DLErrorBits(void);

// -----------------------------------------------------------------------------
uint32_t LSM303DLLastRequestTimestamp(void);

// -----------------------------------------------------------------------------
uint32_t LSM303DLLastUpdateTimestamp(void);

// -----------------------------------------------------------------------------
const float * MagneticVector(void);

// -----------------------------------------------------------------------------
const int16_t * MagnetometerVector(void);


// =============================================================================
// Public functions:

uint32_t LSM303DLInit(void);

// -----------------------------------------------------------------------------
uint32_t ProcessIncomingLSM303DL(void);

// -----------------------------------------------------------------------------
uint32_t RequestLSM303DL(void);


#endif  // LSM303DL_H_
