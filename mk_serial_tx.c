#include "mk_serial_tx.h"

#include "mk_serial_protocol.h"
#include "timing.h"


// =============================================================================
// Private data:

#define STREAM_TIMEOUT (4000)  // ms

static uint32_t tx_request_ = 0x00000000;
static enum MKStream mk_stream_ = MK_STREAM_NONE;
static uint32_t stream_period_ = 0, stream_timer_ = 0, stream_timeout_ = 0;


// =============================================================================
// Private function declarations:

static void SendVersion(void);


// =============================================================================
// Public functions:

// This function sends data that has been requested.
void SendPendingMKSerial(void)
{
  // Handle only one request at a time.
  if (tx_request_)
  {
    // A one-time request has higher priority than a periodic "stream" of data.
    if (tx_request_ & MK_TX_VERSION) SendVersion();
  }
  else if (mk_stream_ && TimestampInPast(stream_timer_))
  {
    // A data stream is active and it is time for another transmission.
    switch (mk_stream_)
    {
      default:
        break;
    }
    stream_timer_ += stream_period_;

    // Prevent timer rollover for small periods.
    if (TimestampInPast(stream_timer_)) stream_timer_ = GetTimestamp();

    // Disable the stream if no request has been renewing received in a while.
    if (TimestampInPast(stream_timeout_)) mk_stream_ = MK_STREAM_NONE;
  }
}

// -----------------------------------------------------------------------------
// This function starts the specified data stream at the specified period. Note
// that this stream has to be renewed periodically by resending the request. If
// no renewing request is received, then the stream will time out after a while.
// Also note that the stream output period will be quantized to the main control
// frequency.
void SetMKDataStream(enum MKStream mk_stream, uint32_t period_10ms)
{
  mk_stream_ = mk_stream;

  uint32_t stream_period = period_10ms * 10;  // ms
  if (!stream_period_)
    stream_timer_ = GetTimestampMillisFromNow(0);  // Start stream immediately
  else if (stream_period < stream_period_)
    stream_timer_ = GetTimestampMillisFromNow(stream_period);
  stream_period_ = stream_period;
  stream_timeout_ = GetTimestampMillisFromNow(STREAM_TIMEOUT);
}

// -----------------------------------------------------------------------------
// This function sets a one-time request for data.
void SetMKTxRequest(enum MKTxBits tx_request)
{
  tx_request_ |= tx_request;
}


// =============================================================================
// Private functions:

static void SendVersion(void)
{
  MKSerialTx(1, 'V', 0, 0, UART_PORT_UART1);
  tx_request_ &= ~MK_TX_VERSION;
}
