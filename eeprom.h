// This file provides a structure and initial conditions for placing persistent
// variables in EEPROM. This method ensures that variables remain in the same
// memory location every for every compilation. This means that EEPROM doesn't
// have to be cleared when the chip is reprogrammed, and previously stored
// values will remain even after programming.

#ifndef EEPROM_H_
#define EEPROM_H_


#include <inttypes.h>


// =============================================================================
// Accessors:

const int16_t * MagnetometerBiasVector(void);

// -----------------------------------------------------------------------------
const float * MagnetometerScaleVector(void);

// -----------------------------------------------------------------------------
void WriteMagnatometerBiasToEEPROM(uint8_t bias[3]);

// -----------------------------------------------------------------------------
void WriteMagnatometerScaleToEEPROM(uint8_t scale[3]);


// =============================================================================
// Public functions:

void ReadEEPROM(void);


#endif  // EEPROM_H_
