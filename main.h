#ifndef MAIN_H_
#define MAIN_H_


typedef void (*Callback)(void);


// =============================================================================
// Public functions:

void FiftyHzInterruptHandler(void);

//------------------------------------------------------------------------------
void FltCtrlInterruptHandler(void);

//------------------------------------------------------------------------------
void NewDataInterruptHandler(void);

// -----------------------------------------------------------------------------
void SetNewDataCallback(Callback callback);


#endif  // MAIN_H_
