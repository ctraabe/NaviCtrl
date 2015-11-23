#ifndef FLT_CTRL_COMMS_H_
#define FLT_CTRL_COMMS_H_


#include <inttypes.h>
#include <stddef.h>


// =============================================================================
// Accessors:

const float * AccelerationVector(void);

// -----------------------------------------------------------------------------
const float * AngularRateVector(void);

// -----------------------------------------------------------------------------
const float * Quat(void);


// =============================================================================
// Public functions:

void FltCtrlCommsInit(void);

// -----------------------------------------------------------------------------
void ReleaseFltCtrlInterrupt(void);

// -----------------------------------------------------------------------------
void SetFltCtrlInterrupt(void);

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_buffer_) by the Rx interrupt handler.
void ProcessIncomingFltCtrlByte(uint8_t byte);


#endif  // FLT_CTRL_COMMS_H_
