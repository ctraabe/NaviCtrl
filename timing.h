#ifndef TIMING_H_
#define TIMING_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

// This function initializes TIMER1 and TIMER3. This timer trigger interrupts
// at 1 kHz and 128 Hz respectively.
void TimingInit(void);

// -----------------------------------------------------------------------------
// This function returns the current timestamp.
uint32_t GetTimestamp(void);

// -----------------------------------------------------------------------------
// This function returns a timestamp corresponding to "t" ms in the future. This
// timestamp can be checked against the current timestamp to see if a certain
// amount of time has passed.
uint32_t GetTimestampMillisFromNow(uint32_t t);

// -----------------------------------------------------------------------------
// This function compares a timestamp to the current timestamp and returns TRUE
// if the timestamp is in the past.
int TimestampInPast(uint32_t t);

// -----------------------------------------------------------------------------
// This function returns the amount of time that has elapsed since the timestamp
// "last_time" has occurred.
uint32_t MillisSinceTimestamp(uint32_t t);

// -----------------------------------------------------------------------------
// This function delays execution of the program for "t" ms. Functions triggered
// by interrupts will still execute during this period.
void Wait(uint32_t w);


#endif  // TIMING_H_
