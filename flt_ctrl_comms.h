#ifndef FLT_CTRL_COMMS_H_
#define FLT_CTRL_COMMS_H_


#include <inttypes.h>
#include <stddef.h>


// =============================================================================
// Accessors:

const volatile float * AccelerationVector(void);

// -----------------------------------------------------------------------------
const volatile float * AngularRateVector(void);

// -----------------------------------------------------------------------------
const volatile float * Quat(void);


// =============================================================================
// Public functions:

void FltCtrlCommsInit(void);

// -----------------------------------------------------------------------------
// This function pulls the interrupt line down for about 1 microsecond.
void NotifyFltCtrl(void);

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_buffer_) by the Rx interrupt handler.
void ProcessIncomingFltCtrlByte(uint8_t byte);

// -----------------------------------------------------------------------------
void SendDataToFltCtrl(void);


#endif  // FLT_CTRL_COMMS_H_
