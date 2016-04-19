#ifndef VISION_H_
#define VISION_H_


#include <inttypes.h>

#include "constants.h"

struct FromVision {
  uint32_t dt;  // microseconds
  uint16_t status;
  float position[3];  // meters
  float quaternion[3];
} __attribute__((packed));


// =============================================================================
// Global data (workaround):

extern struct FromVision g_from_vision;


// =============================================================================
// Accessors:

const float * VisionBodyVelocityVector(void);

// -----------------------------------------------------------------------------
float VisionHeading(void);

// -----------------------------------------------------------------------------
const float * VisionPositionVector(void);

// -----------------------------------------------------------------------------
const float * VisionQuaternionVector(void);

// -----------------------------------------------------------------------------
uint16_t VisionStatus(void);

// -----------------------------------------------------------------------------
const float * VisionVelocityVector(void);

// -----------------------------------------------------------------------------
const struct FromVision * FromVision(void);


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
