#ifndef VISION_H_
#define VISION_H_


#include <inttypes.h>

#include "constants.h"


// =============================================================================
// Accessors:

const float * VisionVelocityVector(void);


// =============================================================================
// Public functions:

void VisionInit(void);

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_buffer_) by the Rx interrupt handler.
uint32_t ProcessIncomingVision(void);

// -----------------------------------------------------------------------------
void VisionUARTHandler(void);


#endif  // VISION_H_
