#ifndef UT_SERIAL_PROTOCOL_H_
#define UT_SERIAL_PROTOCOL_H_


#include <inttypes.h>
#include <stddef.h>

#include "uart.h"


#define UT_START_CHARACTER ('S')
#define UT_HEADER_LENGTH (4)

enum UTSerialComponentID {
  UT_SERIAL_COMPONENT_ID_RICOH = 0,
  UT_SERIAL_COMPONENT_ID_TX1   = 1,
  UT_SERIAL_COMPONENT_ID_RASPI = 2,
};

enum UTSerialNaviCtrlMessageID {
  UT_SERIAL_ID_BEEP_PATTERN = 0,
};

enum UTSerialRaspiMessageID {
  UT_SERRIAL_RASPI_MESSAGE_ID_6DOF_POSITION = 0,
};


// =============================================================================
// Public functions:

// This function collects an incoming byte that is assumed to be part of a
// message encoded in the UTokyo protocol. The return value indicates whether or
// not more bytes are expected. If so, subsequent bytes should also be passed to
// this function.
enum UARTRxMode UTSerialRx(uint8_t byte, uint8_t * data_buffer);

// -----------------------------------------------------------------------------
// This function encodes data into a message using the UTokyo protocol. The
// message must contain at least a destination address and a label. If no
// additional data is necessary, then the source pointer and length can both be
// set to zero.
void UTSerialTx(uint8_t component_id, uint8_t message_id,
  const uint8_t * source, size_t length, enum UARTPort uart_port);


#endif  // UT_SERIAL_PROTOCOL_H_
