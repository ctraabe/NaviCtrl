#ifndef HEADING_H_
#define HEADING_H_


#include <inttypes.h>


// =============================================================================
// Accessors:

float HeadingAngle(void);

// -----------------------------------------------------------------------------
float HeadingCorrectionQuat0(void);

// -----------------------------------------------------------------------------
float HeadingCorrectionQuatZ(void);


// =============================================================================
// Public functions:

void UpdateHeading(void);


#endif  // HEADING_H_
