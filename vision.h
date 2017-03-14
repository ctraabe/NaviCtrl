#ifndef VISION_H_
#define VISION_H_


#include <inttypes.h>
#include <stddef.h>

#include "constants.h"


enum RicohMessageID {
  VISION_MESSAGE_ID_RICOH_VO       = 0x10,
  VISION_MESSAGE_ID_RICOH_OBSTACLE = 0x24,
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
  float linear_resolution;  // (mm)
  float angular_resolution;  // (rad)
  float angular_offset;  // (rad)
  uint8_t number_of_divisions;
  uint8_t obstacle_distance[12];  // * linear_resolution
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
size_t VisionNearestObstacleBin(void);

// -----------------------------------------------------------------------------
float VisionObstacleDistance(size_t bin_index);

// -----------------------------------------------------------------------------
const float * VisionObstacleDistanceArray(void);

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
void ProcessRicohObstacleData(struct RicohObjectDetection * from_ricoh);

// -----------------------------------------------------------------------------
void ProcessRicohVisionData(struct RicohVision * from_ricoh);

// -----------------------------------------------------------------------------
void ProcessTX1VisionData(struct TX1Vision * from_vision);


#endif  // VISION_H_
