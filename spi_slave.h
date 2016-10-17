#ifndef SPI_SLAVE_H_
#define SPI_SLAVE_H_


#include <inttypes.h>
#include <stddef.h>


// =============================================================================
// Public functions:

void SPISlaveInit(void);

// -----------------------------------------------------------------------------
void ProcessIncomingSPISlave(void);

// -----------------------------------------------------------------------------
void SPITxBuffer(uint8_t * buffer, uint8_t tx_length);

// -----------------------------------------------------------------------------
void SPISlaveHandler(void);


#endif  // SPI_SLAVE_H_
