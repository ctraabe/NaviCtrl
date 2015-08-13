#ifndef UBLOX_H_
#define UBLOX_H_


#include <inttypes.h>
#include <stddef.h>


// =============================================================================
// Public functions:

void UBloxInit(void);

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_buffer_) by the Rx interrupt handler. Each byte is passed to the
// appropriate Rx handler, which may place it into the temporary data buffer
// (data_buffer_).
void ProcessIncomingUBlox(void);

// -----------------------------------------------------------------------------
// This function sends the contents of buffer to the u-blox device. This
// function blocks program execution until the entire buffer is sent.
void UBloxTxBuffer(const uint8_t * buffer, size_t length);

// -----------------------------------------------------------------------------
// This function immediately transmits a byte and blocks computation until
// transmission is commenced.
void UBloxTxByte(uint8_t byte);


#endif  // UBLOX_H_
