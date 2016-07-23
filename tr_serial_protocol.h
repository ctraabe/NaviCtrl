#ifndef TR_SERIAL_PROTOCOL_H_
#define TR_SERIAL_PROTOCOL_H_


#include <inttypes.h>
#include <stddef.h>

#include "uart2.h"


// =============================================================================
// Accessors:

uint16_t TRIR(void);

// -----------------------------------------------------------------------------
uint16_t TRSonar(void);

// -----------------------------------------------------------------------------
uint16_t TRCRC(void);


// =============================================================================
// Public functions:

enum UART2RxMode TRSerialRx(uint8_t byte, uint8_t * data_buffer);


#endif  // TR_SERIAL_PROTOCOL_H_
