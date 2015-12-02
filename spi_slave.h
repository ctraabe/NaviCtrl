#ifndef SPI_SLAVE_H_
#define SPI_SLAVE_H_


#include <inttypes.h>
#include <stddef.h>


#define SPI_TX_BUFFER_LENGTH (64)


// =============================================================================
// Public functions:

void SPISlaveInit(void);

// -----------------------------------------------------------------------------
void ProcessIncomingSPISlave(void);

// -----------------------------------------------------------------------------
// This function returns the address of the shared Tx buffer (tx_buffer_) if it
// is available or zero (NULL) if not.
uint8_t * RequestSPITxBuffer(void);

// -----------------------------------------------------------------------------
void SPITxBuffer(uint8_t tx_length);

// -----------------------------------------------------------------------------
void SPISlaveHandler(void);


#endif  // SPI_SLAVE_H_
