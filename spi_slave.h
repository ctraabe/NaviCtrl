#ifndef SPI_SLAVE_H_
#define SPI_SLAVE_H_


#include <inttypes.h>
#include <stddef.h>


// =============================================================================
// Public functions:

void SPISlaveInit(void);

// -----------------------------------------------------------------------------
void SPISlaveHandler(void);

// -----------------------------------------------------------------------------
size_t PrintSensorData(char * ascii, size_t max_length);


#endif  // SPI_SLAVE_H_
