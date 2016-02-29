#ifndef NAVIGATION_H_
#define NAVIGATION_H_


enum NavMode {
  NAV_MODE_OFF = 0,
  NAV_MODE_HOLD = 0x01,
  NAV_MODE_AUTO = 0x02,
  NAV_MODE_HOME = 0x03,
};


// =============================================================================
// Accessors:

const float * NavDeltaPosition(void);

// -----------------------------------------------------------------------------
enum NavMode NavMode(void);

// -----------------------------------------------------------------------------
float TargetHeading(void);

// -----------------------------------------------------------------------------
const float * TargetPosition(void);

// -----------------------------------------------------------------------------
float TransitSpeed(void);


// =============================================================================
// Public functions:

void UpdateNavigation(void);


#endif  // NAVIGATION_H_
