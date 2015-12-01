#ifndef MAIN_H_
#define MAIN_H_


typedef void (*Callback)(void);

enum DataReadyBits {
  DATA_READY_BIT_FC = 1<<0,
  DATA_READY_BIT_GPS = 1<<1,
  DATA_READY_BIT_MAG = 1<<2,
};


// =============================================================================
// Public functions:

void FiftyHzInterruptHandler(void);

//------------------------------------------------------------------------------
void FltCtrlInterruptHandler(void);

//------------------------------------------------------------------------------
void NewDataInterruptHandler(void);

// -----------------------------------------------------------------------------
void SetNewDataCallback(Callback callback);

// -----------------------------------------------------------------------------
void DataReady(enum DataReadyBits data_ready);


#endif  // MAIN_H_
