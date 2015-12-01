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
void OpenLogFile(const char * filename);

// -----------------------------------------------------------------------------
void CloseLogFile(void);

// -----------------------------------------------------------------------------
void NewDataToLogInterruptHandler(void);

// -----------------------------------------------------------------------------
void ProcessLogging(void);


#endif  // LOGGING_H_
