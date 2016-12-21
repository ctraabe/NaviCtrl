#ifndef LOGGING_H_
#define LOGGING_H_


#include <inttypes.h>
#include <stddef.h>

#include "flight_ctrl_comms.h"
#ifdef VISION
  #include "vision.h"
#endif


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
#ifdef VISION
// -----------------------------------------------------------------------------
void LogKalmanData(void);
#endif
// -----------------------------------------------------------------------------
void LogMagnetometerData(void);

// -----------------------------------------------------------------------------
void LogRaspiTimestamp(uint32_t timestamp);

// -----------------------------------------------------------------------------
void LogToFlightCtrlData(const struct ToFlightCtrl * data);
#ifdef VISION
// -----------------------------------------------------------------------------
void LogTX1VisionData(struct TX1Vision * from_tx1);
#endif
// -----------------------------------------------------------------------------
void LogUBXPosLLH(void);

// -----------------------------------------------------------------------------
void LogUBXVelNED(void);

// -----------------------------------------------------------------------------
void LogUBXSol(void);

// -----------------------------------------------------------------------------
void LogUBXTimeUTC(void);

// -----------------------------------------------------------------------------
void ProcessLogging(void);

// -----------------------------------------------------------------------------
void LogWrite(char * buffer, uint32_t length);


#endif  // LOGGING_H_
