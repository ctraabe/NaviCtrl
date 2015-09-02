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
void UBloxUARTHandler(void);


#endif  // UBLOX_H_
