#ifndef NAVIGATION_H_
#define NAVIGATION_H_


#include "constants.h"


enum NavMode {
  NAV_MODE_OFF = 0,
  NAV_MODE_HOLD = 0x01,
  NAV_MODE_AUTO = 0x02,
  NAV_MODE_HOME = 0x03,
};

enum NavError {
  NAV_ERROR_NONE = 0,
  NAV_ERROR_SD_ROUTE_NUMBER,
  NAV_ERROR_SD_WAYPOINT_COUNT,
  NAV_ERROR_SD_DATA_ROWS,
};


// =============================================================================
// Accessors:

float HeadingRate(void);

// -----------------------------------------------------------------------------
float NavDeltaPosition(enum WorldAxes axis);

// -----------------------------------------------------------------------------
enum NavMode NavMode(void);

// -----------------------------------------------------------------------------
float TargetHeading(void);

// -----------------------------------------------------------------------------
float TargetPosition(enum WorldAxes axis);

// -----------------------------------------------------------------------------
float TransitSpeed(void);


// =============================================================================
// Public functions:

void NavigationInit(void);

// -----------------------------------------------------------------------------
void UpdateNavigation(void);


#endif  // NAVIGATION_H_
