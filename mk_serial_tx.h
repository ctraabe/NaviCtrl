#ifndef MK_SERIAL_TX_H_
#define MK_SERIAL_TX_H_


#include <inttypes.h>


enum MKTxBits {
  MK_TX_VERSION = 1<<0,
  MK_TX_MAG = 1<<1,
};

enum MKStream {
  MK_STREAM_NONE = 0,
  MK_STREAM_DEBUG,
  MK_STREAM_MAG,
};


// =============================================================================
// Public functions:

// This function sends data that has been requested.
void SendPendingMKSerial(void);

// -----------------------------------------------------------------------------
// This function starts the specified data stream at the specified period. Note
// that this stream has to be renewed periodically by resending the request. If
// no renewing request is received, then the stream will time out after a while.
// Also note that the stream output period will be quantized to the main control
// frequency.
void SetMKDataStream(enum MKStream mk_stream, uint32_t period_10ms);

// -----------------------------------------------------------------------------
// This function sets a one-time request for data.
void SetMKTxRequest(enum MKTxBits);


#endif  // MK_SERIAL_TX_H_
