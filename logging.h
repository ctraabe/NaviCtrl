#ifndef LOGGING_H_
#define LOGGING_H_


#include <inttypes.h>
#include <stddef.h>


// =============================================================================
// Public functions:

void LoggingInit(void);

// -----------------------------------------------------------------------------
void OpenLogFile(const char * filename);

// -----------------------------------------------------------------------------
void WriteToLog(const char * ascii, size_t length);

// -----------------------------------------------------------------------------
void CloseLogFile(void);

// -----------------------------------------------------------------------------
void ProcessLogging(void);

#endif  // LOGGING_H_
