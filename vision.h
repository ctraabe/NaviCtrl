#ifndef VISION_H_
#define VISION_H_


#include <inttypes.h>

#include "constants.h"


enum RicohMessageID {
  VISION_MESSAGE_ID_RICOH_VO       = 0x10,
  VISION_MESSAGE_ID_RICOH_OBSTACLE = 0x20,
};

struct RaspiVision {
  uint32_t timestamp;  // microseconds
  float position[3];  // meters
  float quaternion[3];
  float position_variance[3];
  uint8_t status;
} __attribute__((packed));

struct RicohObjectDetection {
  uint32_t latency;  // Latency (ms)
  uint32_t capture_time;
  float position[3];
} __attribute__((packed));

struct RicohVision {
  uint32_t latency;  // Latency (ms)
  uint32_t capture_time;
  float velocity[3];  // (mm/frame)
  float quaternion[3];  // [q_x, q_y, q_z]
  float angular_velocity[3];  // (rad/frame)
  float position[3];  // (mm)
  uint16_t reliability;
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
uint32_t VisionDataStale(void);

// -----------------------------------------------------------------------------
uint16_t VisionStatus(void);

// -----------------------------------------------------------------------------
uint32_t VisionTimestamp(void);

// -----------------------------------------------------------------------------
float * VisionVelocityNEDVector(void);


// =============================================================================
// Public functions:

void CheckVisionFreshness(void);

// -----------------------------------------------------------------------------
void ProcessRaspiVisionData(struct RaspiVision * from_raspi);

// -----------------------------------------------------------------------------
void ProcessRicohVisionData(struct RicohVision * from_ricoh);

// -----------------------------------------------------------------------------
void ProcessTX1VisionData(struct TX1Vision * from_vision);


#endif  // VISION_H_
