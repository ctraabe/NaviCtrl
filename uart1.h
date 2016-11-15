#ifndef UART1_H_
#define UART1_H_


#include <inttypes.h>
#include <stddef.h>


// =============================================================================
// Public functions:

void UART1Init(void);

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_buffer_) by the Rx interrupt handler. Each byte is passed to the
// appropriate Rx handler, which may place it into the temporary data buffer
// (data_buffer_).
void ProcessIncomingUART1(void);

// -----------------------------------------------------------------------------
// This function returns the address of the shared Tx buffer (tx_buffer_) if it
// is available or zero (NULL) if not.
uint8_t * RequestUART1TxBuffer(void);

// -----------------------------------------------------------------------------
// This function calls handlers for pending data transmission requests.
void SendPendingUART1(void);

// -----------------------------------------------------------------------------
// This function initiates the transmission of the data in the Tx buffer.
void UART1TxBuffer(size_t tx_length);

// -----------------------------------------------------------------------------
// This function immediately transmits a byte and blocks computation until
// transmission is commenced.
void UART1TxByte(uint8_t byte);

// -----------------------------------------------------------------------------
// This function mimics printf, but puts the result on the UART stream. It also
// adds the end-of-line characters and checks that the character buffer is not
// exceeded. This version blocks program execution until UART is available and
// then further blocks execution until the transmission has competed.
void UART1Printf(const char *format, ...);

// -----------------------------------------------------------------------------
// This function mimics printf, but puts the result on the UART stream. It also
// adds the end-of-line characters and checks that the character buffer is not
// exceeded. This version attempts to get the UART Tx buffer and then initiates
// an interrupt-bases transmission. This function is non-blocking, but may fail
// to get access to the UART Tx buffer.
void UART1PrintfSafe(const char *format, ...);

// -----------------------------------------------------------------------------
uint32_t UART1WaitUntilCompletion(uint32_t time_limit_ms);

// -----------------------------------------------------------------------------
void UART1Handler(void);


#endif  // UART1_H_
