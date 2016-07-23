#ifndef LOGGING_H_
#define LOGGING_H_


#include <inttypes.h>
#include <stddef.h>


// =============================================================================
// Accessors:

uint32_t LoggingActive(void);


// =============================================================================
// Public functions:

void LoggingInit(void);

// -----------------------------------------------------------------------------
void MountLoggingFS(void);

// -----------------------------------------------------------------------------
void UnmountLoggingFS(void);

// -----------------------------------------------------------------------------
void OpenLogFile(const char * filename);

// -----------------------------------------------------------------------------
void CloseLogFile(void);

// -----------------------------------------------------------------------------
void LogFlightControlData(void);

// -----------------------------------------------------------------------------
void LogMagnetometerData(void);

// -----------------------------------------------------------------------------
void LogTRData(void);

// -----------------------------------------------------------------------------
void LogUBXPosLLH(void);

// -----------------------------------------------------------------------------
void LogUBXVelNED(void);

// -----------------------------------------------------------------------------
void LogUBXSol(void);

// -----------------------------------------------------------------------------
void LogUBXTimeUTC(void);

// -----------------------------------------------------------------------------
void LogVisionData(void);

// -----------------------------------------------------------------------------
void ProcessLogging(void);

// -----------------------------------------------------------------------------
void LogWrite(char * buffer, uint32_t length);


#endif  // LOGGING_H_
