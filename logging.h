#ifndef LOGGING_H_
#define LOGGING_H_


#include <inttypes.h>
#include <stddef.h>


enum DataReadyBits {
  DATA_READY_BIT_GPS = 1<<0,
  DATA_READY_BIT_MAG = 1<<1,
  DATA_READY_BIT_FC = 1<<2,
};


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
void DataReadyToLog(enum DataReadyBits data_ready);

// -----------------------------------------------------------------------------
void NewDataInterruptHandler(void);

// -----------------------------------------------------------------------------
void ProcessLogging(void);


#endif  // LOGGING_H_
