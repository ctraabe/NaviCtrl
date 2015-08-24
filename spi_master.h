#ifndef SPI_MASTER_H_
#define SPI_MASTER_H_


#include <inttypes.h>
#include <stddef.h>


#define SPI_MASTER_TX_BUFFER_LENGTH (128)


// =============================================================================
// Public functions:

void SPIMasterInit(void);

// -----------------------------------------------------------------------------
void SPIMasterSetBaud(uint32_t baud_rate);

// -----------------------------------------------------------------------------
uint32_t SPIMasterGetByte(uint8_t * byte);

// -----------------------------------------------------------------------------
// This function returns the address of the shared Tx buffer (tx_buffer_) if it
// is available or zero (NULL) if not.
uint8_t * RequestSPIMasterTxBuffer(void);

// -----------------------------------------------------------------------------
// This function essentially flushes rx_fifo_.
void SPIMasterResetRxFIFO(void);

// -----------------------------------------------------------------------------
// This function initiates the transmission of the data in a tx_buffer_ and/or
// data reception. An number of bytes exchanged will be the maximum of rx_length
// and tx_length. The transmission will start with tx_length bytes from
// tx_buffer_ and will continue with null (0xFF) bytes if tx_length is less than
// rx_length. For reception, the first rx_length bytes will be put into rx_fifo_
// and subsequent bytes will be ignored if rx_length is less than tx_length.
void SPIMasterStart(uint8_t rx_length, uint8_t tx_length);

// -----------------------------------------------------------------------------
uint32_t SPIMasterWaitUntilCompletion(uint32_t time_limit_ms);


#endif  // SPI_MASTER_H_
