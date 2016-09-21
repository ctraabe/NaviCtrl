#include "ut_serial_rx.h"

#include "ut_serial_protocol.h"


// =============================================================================
// Public functions:

// This function handles the response to data that has been received in the
// UTokyo protocol.
void HandleUTRx(uint8_t id, const uint8_t * data_buffer)
{
  switch (id)
  {
    default:
      // TODO: remove this
      id = data_buffer[0];
      break;
  }
}
