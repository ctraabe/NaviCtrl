#ifndef NAVIGATION_H_
#define NAVIGATION_H_


#include <inttypes.h>

#include "constants.h"


// TODO: correct this for WGS84 model
#define R_EARTH (6378137.0)  // meters
#define UBX_LATITUDE_TO_METERS (1.0e-7 * M_PI / 180.0 * R_EARTH)

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

float CurrentHeading(void);

// -----------------------------------------------------------------------------
float CurrentPosition(enum WorldAxes axis);

// -----------------------------------------------------------------------------
int32_t GPSHome(enum GeoAxes axis);

// -----------------------------------------------------------------------------
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

#ifndef VISION
// -----------------------------------------------------------------------------
void SetGPSHome(void);
#endif


#endif  // NAVIGATION_H_
