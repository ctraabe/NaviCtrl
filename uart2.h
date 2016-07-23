#ifndef UART2_H_
#define UART2_H_


#include <inttypes.h>


#define UART2_RX_FIFO_LENGTH (1 << 7)  // 2^7 = 128
#define UART2_DATA_BUFFER_LENGTH (70)

enum UART2RxMode {
  UART2_RX_MODE_IDLE = 0,
  UART2_RX_MODE_TR_ONGOING,
};


// =============================================================================
// Accessors:

const struct UBXPosLLH * UBXPosLLH(void);

// -----------------------------------------------------------------------------
const struct UBXVelNED * UBXVelNED(void);

// -----------------------------------------------------------------------------
const struct UBXSol * UBXSol(void);

// -----------------------------------------------------------------------------
const struct UBXTimeUTC * UBXTimeUTC(void);


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
// This function immediately transmits a byte and blocks computation until
// transmission is commenced.
void UART2TxByte(uint8_t byte);

// -----------------------------------------------------------------------------
void UART2Handler(void);


#endif  // UART2_H_
