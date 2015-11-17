#ifndef SPI_SLAVE_H_
#define SPI_SLAVE_H_


#include <inttypes.h>
#include <stddef.h>


// =============================================================================
// Accessors:

const float * AccelerationVector(void);

// -----------------------------------------------------------------------------
const float * AngularRateVector(void);

// -----------------------------------------------------------------------------
const float * Quat(void);


// =============================================================================
// Public functions:

void SPISlaveInit(void);

// -----------------------------------------------------------------------------
void SPISlaveHandler(void);

// -----------------------------------------------------------------------------
void ProcessIncomingSPISlave(void);


#endif  // SPI_SLAVE_H_
