#ifndef UART2_H_
#define UART2_H_


#include <inttypes.h>
#include <stddef.h>


#define UART2_RX_FIFO_LENGTH (1 << 7)  // 2^7 = 128
#define UART2_DATA_BUFFER_LENGTH (32)
#define UART2_TX_BUFFER_LENGTH (32)

enum UART2RxMode {
  UART2_RX_MODE_IDLE = 0,
  UART2_RX_MODE_TR_ONGOING,
  UART2_RX_MODE_UT_ONGOING,
};


// =============================================================================
// Public functions:

void UART2Init(void);

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_fifo_) by the Rx interrupt handler. Each byte is passed to the
// appropriate Rx handler, which may place it into the temporary data buffer
// (data_buffer_).
void ProcessIncomingUART2(void);

// -----------------------------------------------------------------------------
// This function returns the address of the shared Tx buffer (tx_buffer_) if it
// is available or zero (NULL) if not.
uint8_t * RequestUART2TxBuffer(void);

// -----------------------------------------------------------------------------
// This function initiates the transmission of the data in the Tx buffer.
void UART2TxBuffer(size_t tx_length);

// -----------------------------------------------------------------------------
// This function immediately transmits a byte and blocks computation until
// transmission is commenced.
void UART2TxByte(uint8_t byte);

// -----------------------------------------------------------------------------
void UART2Handler(void);


#endif  // UART2_H_
