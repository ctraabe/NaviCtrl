// This file employs the FatFS library to manage files and logging to an SD
// card. Note that FatFS writes in 512-byte blocks, and program execution is
// blocked until the write is completed. Therefore, it is important that FatFS
// communication (e.g. f_open, f_read, f_write) be done at the lowest priority
// possible so as not to block other important functions. Note that a 512-byte
// exchange takes approximately 5 ms at SPI using 1 MHz clock.

#include "logging.h"

#include <stdio.h>

#include "91x_lib.h"
#include "crc16.h"
#include "ff.h"  // from libfatfs
#include "flight_ctrl_comms.h"
#include "lsm303dl.h"
#include "sd_card.h"
#include "spi_slave.h"
#include "uart1.h"
#include "union_types.h"
#include "ublox.h"
#ifdef VISION
  #include "kalman_filter.h"
#endif
// TODO: remove
#include "led.h"


// =============================================================================
// Private data:

#define LOG_FIFO_LENGTH (512)
#define MAX_FILENAME_LENGTH (80)

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
  return logging_active_ && file_.fs;
}


// =============================================================================
// Public functions:

void OpenLogFile(const char * filename)
{
  if (!SDCardFSMounted()) return;

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
void LogFromFlightCtrlData(void)
{
  if (SDCardNotPresent() || !file_.fs) return;

  union U16Bytes temp;
  temp.bytes[0] = 0xB5;
  temp.bytes[1] = 0x52;
  WriteToFIFO((char *)temp.bytes, 2);
  WriteToFIFO((char *)FromFlightCtrl(), sizeof(struct FromFlightCtrl));
  temp.u16 = FromFlightCtrlCRC();
  WriteToFIFO((char *)temp.bytes, 2);
}

// -----------------------------------------------------------------------------
#ifdef VISION
void LogKalmanData(void)
{
  if (SDCardNotPresent() || !file_.fs) return;

  union U16Bytes temp;
  temp.bytes[0] = 0x88;
  temp.bytes[1] = 0x99;
  WriteToFIFO((char *)temp.bytes, 2);
  WriteToFIFO((char *)KalmanVelocityVector(), sizeof(float) * 3);
  temp.u16 = CRCCCITT((uint8_t *)KalmanVelocityVector(), sizeof(float) * 3);
  WriteToFIFO((char *)temp.bytes, 2);
}
#endif
// -----------------------------------------------------------------------------
void LogMagnetometerData(void)
{
  if (SDCardNotPresent() || !file_.fs) return;

  union U16Bytes temp;
  temp.bytes[0] = 0xAA;
  temp.bytes[1] = 0xBB;
  WriteToFIFO((char *)temp.bytes, 2);
  WriteToFIFO((char *)MagneticVector(), sizeof(float)*3);
  temp.u16 = CRCCCITT((uint8_t *)MagneticVector(), sizeof(float)*3);
  WriteToFIFO((char *)temp.bytes, 2);
}

// -----------------------------------------------------------------------------
void LogRaspiTimestamp(uint32_t timestamp)
{
  if (SDCardNotPresent() || !file_.fs) return;

  union U16Bytes temp;
  temp.bytes[0] = 0x56;
  temp.bytes[1] = 0x56;
  WriteToFIFO((char *)temp.bytes, 2);
  WriteToFIFO((char *)&timestamp, sizeof(timestamp));
  temp.u16 = CRCCCITT((uint8_t *)&timestamp, sizeof(timestamp));
  WriteToFIFO((char *)temp.bytes, 2);
}

// -----------------------------------------------------------------------------
void LogToFlightCtrlData(const struct ToFlightCtrl * data)
{
  if (SDCardNotPresent() || !file_.fs) return;

  union U16Bytes temp;
  temp.bytes[0] = 0xEE;
  temp.bytes[1] = 0xFF;
  WriteToFIFO((char *)temp.bytes, 2);
  WriteToFIFO((char *)data, sizeof(struct ToFlightCtrl));
}
#ifdef VISION
// -----------------------------------------------------------------------------
void LogTX1VisionData(struct TX1Vision * from_tx1)
{
  if (SDCardNotPresent() || !file_.fs) return;

  union U16Bytes temp;
  temp.bytes[0] = 0xCC;
  temp.bytes[1] = 0xDD;
  WriteToFIFO((char *)temp.bytes, 2);
  WriteToFIFO((char *)from_tx1, sizeof(struct TX1Vision));
  temp.u16 = CRCCCITT((uint8_t *)from_tx1, sizeof(struct TX1Vision));
  WriteToFIFO((char *)temp.bytes, 2);
}
#endif
// -----------------------------------------------------------------------------
void LogUBXPosLLH(void)
{
  if (SDCardNotPresent() || !file_.fs) return;

  uint16_t header = 0x1212;
  WriteToFIFO((char *)&header, 2);
  WriteToFIFO((char *)UBXPosLLH(), sizeof(struct UBXPosLLH));
  uint16_t crc = CRCCCITT((uint8_t *)UBXPosLLH(), sizeof(struct UBXPosLLH));
  WriteToFIFO((char *)&crc, 2);
}

// -----------------------------------------------------------------------------
void LogUBXVelNED(void)
{
  if (SDCardNotPresent() || !file_.fs) return;

  uint16_t header = 0x2323;
  WriteToFIFO((char *)&header, 2);
  WriteToFIFO((char *)UBXVelNED(), sizeof(struct UBXVelNED));
  uint16_t crc = CRCCCITT((uint8_t *)UBXVelNED(), sizeof(struct UBXVelNED));
  WriteToFIFO((char *)&crc, 2);
}

// -----------------------------------------------------------------------------
void LogUBXSol(void)
{
  if (SDCardNotPresent() || !file_.fs) return;

  uint16_t header = 0x3434;
  WriteToFIFO((char *)&header, 2);
  WriteToFIFO((char *)UBXSol(), sizeof(struct UBXSol));
  uint16_t crc = CRCCCITT((uint8_t *)UBXSol(), sizeof(struct UBXSol));
  WriteToFIFO((char *)&crc, 2);
}

// -----------------------------------------------------------------------------
void LogUBXTimeUTC(void)
{
  if (SDCardNotPresent() || !file_.fs) return;

  uint16_t header = 0x4545;
  WriteToFIFO((char *)&header, 2);
  WriteToFIFO((char *)UBXTimeUTC(), sizeof(struct UBXTimeUTC));
  uint16_t crc = CRCCCITT((uint8_t *)UBXTimeUTC(), sizeof(struct UBXTimeUTC));
  WriteToFIFO((char *)&crc, 2);
}

// -----------------------------------------------------------------------------
void ProcessLogging(void)
{
  if (!SDCardFSMounted())
  {
    logging_active_ = 0;
    return;
  }
#ifndef LOGGING_BUTTON
  if (!FlightCtrlCommsOngoing())
  {
    if ((FlightCtrlState() & FC_STATE_BIT_MOTORS_RUNNING) && !logging_active_)
      OpenLogFile(0);
    if (!(FlightCtrlState() & FC_STATE_BIT_MOTORS_RUNNING) && logging_active_)
      CloseLogFile();
  }
#endif
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
        snprintf(filename_, MAX_FILENAME_LENGTH, "LOG%04lu.BIN", ++i);
        // UART1Printf("Trying %s", filename_);
        file_status_ = f_open(&file_, filename_, FA_WRITE | FA_CREATE_NEW);
      } while ((file_status_ == FR_EXIST) && (i < 1000));
    }
    else
    {
      file_status_ = f_open(&file_, filename_, FA_WRITE | FA_CREATE_NEW);
    }

    if (file_status_ == FR_OK)
    {
      UART1Printf("logging: opened file %s", filename_);
    }
    else
    {
      UART1Printf("logging: f_open returned error code: 0x%02X", file_status_);
      file_status_ = f_close(&file_);
      logging_active_ = 0;
      return;
    }
    // GreenLEDOff();
    RedLEDOn();
  }

  // Make sure that the file is open.
  if (!file_.fs)
  {
    logging_active_ = 0;
    return;
  }

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
    if (file_status_ == FR_OK)
    {
      UART1Printf("logging: closed file %s", filename_);
    }
    else
    {
      UART1Printf("logging: f_close returned error code: 0x%02X", file_status_);
    }
    // GreenLEDOff();
    RedLEDOff();
  }
}

// -----------------------------------------------------------------------------
void LogWrite(char * buffer, uint32_t length)
{
  if (!SDCardFSMounted()) return;
  UINT n_bytes_written;
  file_status_ = f_write(&file_, buffer, length, &n_bytes_written);
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
