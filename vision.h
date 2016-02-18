#ifndef VISION_H_
#define VISION_H_


#include <inttypes.h>

#include "constants.h"


// =============================================================================
// Accessors:

const float * VisionAngularVelocityVector(void);

// -----------------------------------------------------------------------------
const float * VisionBodyVelocityVector(void);

// -----------------------------------------------------------------------------
uint16_t VisionCaptureTime(void);

// -----------------------------------------------------------------------------
const float * VisionInertialVelocityVector(void);

// -----------------------------------------------------------------------------
const float * VisionPositionVector(void);

// -----------------------------------------------------------------------------
const float * VisionQuaternionVector(void);

// -----------------------------------------------------------------------------
uint16_t VisionReliability(void);


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
