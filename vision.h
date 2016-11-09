#ifndef VISION_H_
#define VISION_H_


#include <inttypes.h>

#include "constants.h"


enum VisionMessageID {
  VISION_MESSAGE_ID_RICOH_VO       = 0x10,
  VISION_MESSAGE_ID_RICOH_OBSTACLE = 0x20,
};

struct FromVision {
  uint32_t latency;  // Latency (ms)
  uint32_t capture_time;
  float velocity[3];  // (mm/frame)
  float quaternion[3];  // [q_x, q_y, q_z]
  float angular_velocity[3];  // (rad/frame)
  float position[3];  // (mm)
  uint16_t reliability;
} __attribute__((packed));

struct RicohObjectDetection {
  uint32_t latency;  // Latency (ms)
  uint32_t capture_time;
  float position[3];
} __attribute__((packed));

enum VisionErrorBits {
  VISION_ERROR_BIT_STALE = 1<<0,
};


// =============================================================================
// Accessors:

const struct FromVision * FromVision(void);

// -----------------------------------------------------------------------------
const float * VisionAngularVelocityVector(void);

// -----------------------------------------------------------------------------
const float * VisionBodyVelocityVector(void);

// -----------------------------------------------------------------------------
uint16_t VisionCaptureTime(void);

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
void CheckVisionFreshness(void);

// -----------------------------------------------------------------------------
void VisionUARTHandler(void);


#endif  // VISION_H_
