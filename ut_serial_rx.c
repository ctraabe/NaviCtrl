#include "ut_serial_rx.h"

#include "ut_serial_protocol.h"
#include "vision.h"


// =============================================================================
// Public functions:

// This function handles the response to data that has been received in the
// UTokyo protocol.
void HandleUTRx(uint8_t component_id, uint8_t message_id,
  const uint8_t * data_buffer)
{
  switch (component_id)
  {
    case UT_SERIAL_COMPONENT_ID_RICOH:
      switch (message_id)
      {
        case VISION_MESSAGE_ID_RICOH_VO:
          ProcessRicohVisionData((struct RicohVision *)data_buffer);
        default:
          break;
      }
      break;
    case UT_SERIAL_COMPONENT_ID_TX1:
      ProcessTX1VisionData((struct TX1Vision *)data_buffer);
      break;
    case UT_SERIAL_COMPONENT_ID_RASPI:
      ProcessRaspiVisionData((struct RaspiVision *)data_buffer);
      break;
    default:
      break;
  }
}
