#include "mk_serial_rx.h"

#include "91x_lib.h"
#include "mk_serial_protocol.h"
#include "mk_serial_tx.h"
#include "timing.h"


// =============================================================================
// Private function declarations:

static void ResetToBootloader(void);


// =============================================================================
// Public functions:

// This function handles the response to data that has been received in the
// MikroKopter protocol.
void HandleMKRx(uint8_t address, uint8_t label, uint8_t * data_buffer)
{
  // First check for the following address independent messages.
  switch (label)
  {
    case 'i':  // Request MK data stream
      SetMKDataStream(MK_STREAM_MAG, 0);
      break;
    case 'r':  // Request data stream reset
      SetMKDataStream(MK_STREAM_NONE, 0);
      break;
    case 'd':  // Request MK debug stream
      SetMKDataStream(MK_STREAM_DEBUG, data_buffer[0]);
      break;
    case 'v':  // Request firmware version
      SetMKTxRequest(MK_TX_VERSION);
      break;
    case 'R':  // Reset to bootloader
      ResetToBootloader();
      break;
    default:
      // Check for FlightCtrl specific messages.
      if (address == MK_SERIAL_FC_ADDRESS)
      {
      }
      break;
  }
}

// -----------------------------------------------------------------------------
static void ResetToBootloader(void)
{
  VIC_DeInit();

  SCU_APBPeriphClockConfig(__WDG, ENABLE);

  WDG_InitTypeDef WDG_InitStructure;
  WDG_InitStructure.WDG_ClockSource=WDG_ClockSource_Apb;
  WDG_InitStructure.WDG_Prescaler = 0xFF;
  WDG_InitStructure.WDG_Preload = 0xFFFF;

  WDG_Init(&WDG_InitStructure);
  WDG_StartWatchdogMode();

  for (;;) continue;
}
