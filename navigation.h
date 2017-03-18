#ifndef NAVIGATION_H_
#define NAVIGATION_H_


#include <inttypes.h>

#include "constants.h"
#include "obstacle_avoidance.h"
#include "sensor_enumeration.h"


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

enum WaypointTypeBits {
  WP_TYPE_BIT_VISION        = 1<<0,
  WP_TYPE_BIT_GEODETIC      = 1<<1,
  WP_TYPE_BIT_DISABLE_AVOID = 1<<2,
};

enum WaypointType {
  WP_TYPE_GPS_RELATIVE    = 0,
  WP_TYPE_VISION_RELATIVE = WP_TYPE_BIT_VISION,
  WP_TYPE_GPS_GEODETIC    = WP_TYPE_BIT_GEODETIC,
  WP_TYPE_VISION_GEODETIC = WP_TYPE_BIT_VISION | WP_TYPE_BIT_GEODETIC,
};


// =============================================================================
// Accessors:

enum SensorBits ActiveNavSensorBits(void);

// -----------------------------------------------------------------------------
uint32_t AvoidanceMode(void);

// -----------------------------------------------------------------------------
uint32_t CurrentWaypoint(void);

// -----------------------------------------------------------------------------
float HeadingRate(void);

// -----------------------------------------------------------------------------
float NavDeltaPosition(enum WorldAxes axis);

// -----------------------------------------------------------------------------
enum NavMode NavMode(void);

// -----------------------------------------------------------------------------
uint32_t Route(void);

// -----------------------------------------------------------------------------
float TargetHeading(void);

// -----------------------------------------------------------------------------
float TargetPosition(enum WorldAxes axis);

// -----------------------------------------------------------------------------
float TransitSpeed(void);

// -----------------------------------------------------------------------------
float UBXLongitudeToMeters(void);


// =============================================================================
// Public functions:

void NavigationInit(void);

// -----------------------------------------------------------------------------
void AvoidanceUpdate(enum AvoidanceWidth avoidance_width_enum);

// -----------------------------------------------------------------------------
void UpdateNavigation(void);

// -----------------------------------------------------------------------------
void SetGeodeticHome(void);

// -----------------------------------------------------------------------------
void UBXToMeters(const int32_t ubx_geodetic[2], float ne[2]);


#endif  // NAVIGATION_H_
