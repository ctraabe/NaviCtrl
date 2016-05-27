#ifndef VISION_H_
#define VISION_H_


#include <inttypes.h>

#include "constants.h"


struct FromVision {
  uint32_t dt;  // microseconds
  float position[3];  // meters
  float quaternion[3];
  uint16_t status;
} __attribute__((packed));


// =============================================================================
// Accessors:

const float * VisionBodyVelocityVector(void);

// -----------------------------------------------------------------------------
float VisionDT(void);

// -----------------------------------------------------------------------------
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
