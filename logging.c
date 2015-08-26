#include "logging.h"

#include "ff.h"  // FatFs
#include "uart.h"
#include "sd_card.h"


// =============================================================================
// Private data:

#define LOG_FIFO_LENGTH (512)

static FATFS fat_fs_;
static FIL file_;
static FRESULT file_status_ = FR_NOT_ENABLED;
static char log_fifo_[LOG_FIFO_LENGTH];
static volatile size_t log_fifo_head_ = 0;


// =============================================================================
// Private function declarations:


// =============================================================================
// Public functions:

void LoggingInit(void)
{
  SDCardInit();

  file_status_ = f_mount(&fat_fs_, "", 0);
  if (SDCardNotPresent()) UARTPrintf("logging: SD card not present");
}

// -----------------------------------------------------------------------------
void OpenLogFile(const char * filename)
{
  if (file_status_ != FR_OK) return;
  file_status_ = f_open(&file_, filename, FA_WRITE | FA_CREATE_ALWAYS);
}

// -----------------------------------------------------------------------------
void WriteToLog(const char * ascii, size_t length)
{
  if ((file_status_ != FR_OK) || (length == 0)) return;
  do
  {
    log_fifo_head_ = (log_fifo_head_ + 1) % LOG_FIFO_LENGTH;
    log_fifo_[log_fifo_head_] = *ascii++;
  } while (--length);
}

// -----------------------------------------------------------------------------
void CloseLogFile(void)
{
  file_status_ = f_close(&file_);
}

// -----------------------------------------------------------------------------
void ProcessLogging(void)
{
  if (file_status_ != FR_OK) return;

  // Make a copy of the volatile head index.
  size_t log_fifo_head = log_fifo_head_;

  static size_t log_fifo_tail = 0;
  UINT n_bytes_written;
  // If the head has wrapped around to the front of the FIFO, start by writing
  // the end of the FIFO.
  if (log_fifo_head < log_fifo_tail)
    file_status_ = f_write(&file_, &log_fifo_[log_fifo_tail], LOG_FIFO_LENGTH
      - log_fifo_tail, &n_bytes_written);

  if (log_fifo_head > log_fifo_tail)
    file_status_ = f_write(&file_, &log_fifo_[log_fifo_tail], log_fifo_head
      - log_fifo_tail, &n_bytes_written);
}
