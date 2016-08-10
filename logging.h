#ifndef LOGGING_H_
#define LOGGING_H_


#include <inttypes.h>
#include <stddef.h>

#include "flight_ctrl_comms.h"


// =============================================================================
// Accessors:

uint32_t LoggingActive(void);


// =============================================================================
// Public functions:

void OpenLogFile(const char * filename);

// -----------------------------------------------------------------------------
void CloseLogFile(void);

// -----------------------------------------------------------------------------
void LogFromFlightCtrlData(void);

// -----------------------------------------------------------------------------
void LogKalmanData(void);

// -----------------------------------------------------------------------------
void LogMagnetometerData(void);

// -----------------------------------------------------------------------------
void LogToFlightCtrlData(const struct ToFlightCtrl * data);

// -----------------------------------------------------------------------------
void LogVisionData(void);

// -----------------------------------------------------------------------------
void ProcessLogging(void);

// -----------------------------------------------------------------------------
void LogWrite(char * buffer, uint32_t length);


#endif  // LOGGING_H_
