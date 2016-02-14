#ifndef MAIN_H_
#define MAIN_H_


typedef void (*Callback)(void);


// =============================================================================
// Public functions:

void FiftyHzInterruptHandler(void);

//------------------------------------------------------------------------------
void FlightCtrlInterruptHandler(void);

//------------------------------------------------------------------------------
void NewDataInterruptHandler(void);

// -----------------------------------------------------------------------------
void SetNewDataCallback(Callback callback);

// -----------------------------------------------------------------------------
void SetFlightCtrlInterrupt(void);


#endif  // MAIN_H_
