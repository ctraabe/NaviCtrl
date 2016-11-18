#ifndef VISION_H_
#define VISION_H_


#include <inttypes.h>

#include "constants.h"


struct RaspiVision {
  uint32_t timestamp;  // microseconds
  float position[3];  // meters
  float velocity[3];
  float heading;
  float position_sigma[3];
  float velocity_sigma[3];
  uint8_t status;
} __attribute__((packed));

struct TX1Vision {
  uint32_t timestamp;  // microseconds
  float position[3];  // meters
  float quaternion[3];
  uint16_t status;
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


// =============================================================================
// Public functions:

void CheckVisionFreshness(void);

// -----------------------------------------------------------------------------
void ProcessRaspiVisionData(struct RaspiVision * from_raspi);

// -----------------------------------------------------------------------------
void ProcessTX1VisionData(struct TX1Vision * from_vision);


#endif  // VISION_H_
