#ifndef VISION_H_
#define VISION_H_


#include <inttypes.h>

#include "constants.h"


struct FromVision {
  uint16_t timestamp[2];
} __attribute__((packed));

enum VisionErrorBits {
  VISION_ERROR_BIT_STALE = 1<<0,
};


// =============================================================================
// Accessors:

float VisionHeading(void);

// -----------------------------------------------------------------------------
float VisionPosition(enum WorldAxes axis);

// -----------------------------------------------------------------------------
const float * VisionPositionVector(void);

// -----------------------------------------------------------------------------
const float * VisionQuaternionVector(void);

// -----------------------------------------------------------------------------
uint16_t VisionStatus(void);

// -----------------------------------------------------------------------------
uint32_t VisionTimestamp(void);

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
void CheckVisionFreshness(void);

// -----------------------------------------------------------------------------
void VisionUARTHandler(void);


#endif  // VISION_H_
