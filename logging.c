// This file employs the FatFS library to manage files and logging to an SD
// card. Note that FatFS writes in 512-byte blocks, and program execution is
// blocked until the write is completed. Therefore, it is important that FatFS
// communication (e.g. f_open, f_read, f_write) be done at the lowest priority
// possible so as not to block other important functions. Note that a 512-byte
// exchange takes approximately 5 ms at SPI using 1 MHz clock.

#include "logging.h"

#include <stdio.h>

#include "91x_lib.h"
#include "ff.h"  // FatFs
#include "lsm303dl.h"
#include "uart.h"
#include "sd_card.h"
#include "spi_slave.h"


// =============================================================================
// Private data:

#define LOG_FIFO_LENGTH (512)
#define MAX_FILENAME_LENGTH (80)

static FATFS fat_fs_ = { 0 };
static FIL file_ = { 0 };
static FRESULT file_status_ = FR_INVALID_DRIVE;
static char filename_[MAX_FILENAME_LENGTH];
static volatile char log_fifo_[LOG_FIFO_LENGTH];
static volatile size_t log_fifo_head_ = 0;
static volatile uint32_t logging_active_ = 0;


// =============================================================================
// Private function declarations:

static void WriteToFIFO(const char * ascii, size_t length);


// =============================================================================
// Accessors:

uint32_t LoggingActive(void)
{
  return logging_active_;
}


// =============================================================================
// Public functions:

void LoggingInit(void)
{
  SDCardInit();

  file_status_ = f_mount(&fat_fs_, "", 0);
  if (SDCardNotPresent()) UARTPrintf("logging: SD card not present");
  UARTWaitUntilCompletion(100);
}

// -----------------------------------------------------------------------------
void OpenLogFile(const char * filename)
{
  if (file_status_ == FR_INVALID_DRIVE) return;

  if (filename)
  {
    // Copy the filename.
    char * filename_ptr = filename_;
    size_t max_filename_length = MAX_FILENAME_LENGTH;
    do
    {
      *filename_ptr++ = *filename;
    } while (*filename++ && --max_filename_length);
  }
  else
  {
    filename_[0] = 0;
  }

  logging_active_ = 1;
}

// -----------------------------------------------------------------------------
void CloseLogFile(void)
{
  logging_active_ = 0;
}

// -----------------------------------------------------------------------------
void NewDataToLogInterruptHandler(void)
{
  // VIC_SWITCmd(EXTIT1_ITLine, DISABLE);

  if (!logging_active_) return;

  char ascii[80];

  // if (data_ready_ & DATA_READY_BIT_MAG)
  // {
  //   data_ready_ &= ~DATA_READY_BIT_MAG;

    size_t length = snprintf(ascii, 80, "mag,%i,%i,%i\r\n",
      MagnetometerVector()[0], MagnetometerVector()[1],
      MagnetometerVector()[2]);
    WriteToFIFO(ascii, length);
  // }
}

// -----------------------------------------------------------------------------
void ProcessLogging(void)
{
  if (SDCardNotPresent()) return;

  // Open the file if logging is supposed to be active but the file is closed.
  if (logging_active_ && !file_.fs)
  {
    // GreenLEDOn();
    if (filename_[0] == 0)
    {
      // Try to open a default filename.
      uint32_t i = 0;
      do
      {
        snprintf(filename_, MAX_FILENAME_LENGTH, "LOG%04lu.CSV", ++i);
        // UARTPrintf("Trying %s", filename_);
        file_status_ = f_open(&file_, filename_, FA_WRITE | FA_CREATE_NEW);
      } while ((file_status_ == FR_EXIST) && (i < 1000));
    }
    else
    {
      file_status_ = f_open(&file_, filename_, FA_WRITE | FA_CREATE_NEW);
    }

    if (file_status_ != FR_OK)
    {
      // UARTPrintf("logging: f_open returned error code: 0x%02X", file_status_);
      file_status_ = f_close(&file_);
      logging_active_ = 0;
      return;
    }
    // GreenLEDOff();
    // RedLEDOn();
  }

  // Make sure that the file is open.
  if (!file_.fs) return;

  // Make a copy of the volatile head index.
  size_t log_fifo_head = log_fifo_head_;

  static size_t log_fifo_tail = 0;
  UINT n_bytes_written;

  // If the head has wrapped around to the front of the FIFO, start by writing
  // the end of the FIFO.
  if (log_fifo_head < log_fifo_tail)
  {
    // GreenLEDOn();
    file_status_ = f_write(&file_, (char *)&log_fifo_[log_fifo_tail],
      LOG_FIFO_LENGTH - log_fifo_tail, &n_bytes_written);
    log_fifo_tail = 0;
    // GreenLEDOff();
  }

  if (log_fifo_tail < log_fifo_head)
  {
    // GreenLEDOn();
    file_status_ = f_write(&file_, (char *)&log_fifo_[log_fifo_tail],
      log_fifo_head - log_fifo_tail, &n_bytes_written);
    log_fifo_tail = log_fifo_head;
    // GreenLEDOff();
  }

  // Close the file if logging is not supposed to be active anymore.
  if (!logging_active_)
  {
    // GreenLEDOn();
    file_status_ = f_close(&file_);
    // UARTPrintf("logging: f_close returned error code: 0x%02X", file_status_);
    // GreenLEDOff();
    // RedLEDOff();
  }
}


// =============================================================================
// Private functions:

static void WriteToFIFO(const char * ascii, size_t length)
{
  if ((file_status_ != FR_OK) || !file_.fs || (length == 0)) return;
  do
  {
    log_fifo_[log_fifo_head_] = *ascii++;
    log_fifo_head_ = (log_fifo_head_ + 1) % LOG_FIFO_LENGTH;
  } while (--length);
}
