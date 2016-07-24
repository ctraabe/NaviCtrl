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
#include "flight_ctrl_comms.h"
#include "lsm303dl.h"
#include "sd_card.h"
#include "spi_slave.h"
#include "tr_serial_protocol.h"
#include "uart.h"
#include "union_types.h"
// TODO: remove
#include "crc16.h"
#include "led.h"
#include "ublox.h"
#include "vision.h"


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
  return logging_active_ && file_.fs;
}


// =============================================================================
// Public functions:

void LoggingInit(void)
{
  SDCardInit();
  if (SDCardNotPresent())
    UARTPrintf("logging: SD card not present");
  else
    MountLoggingFS();
}

// -----------------------------------------------------------------------------
void MountLoggingFS(void)
{
  file_status_ = f_mount(&fat_fs_, "", 1);
}

// -----------------------------------------------------------------------------
void UnmountLoggingFS(void)
{
  file_status_ = f_mount(NULL, "", 0);
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
void LogFlightControlData(void)
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
void LogMagnetometerData(void)
{
  if (SDCardNotPresent() || !file_.fs) return;

  union U16Bytes temp;
  temp.bytes[0] = 0xAA;
  temp.bytes[1] = 0xBB;
  WriteToFIFO((char *)temp.bytes, 2);
  WriteToFIFO((char *)MagnetometerVector(), 2*3);
  temp.u16 = CRCCCITT((uint8_t *)MagnetometerVector(), 2*3);
  WriteToFIFO((char *)temp.bytes, 2);
}

// -----------------------------------------------------------------------------
void LogTRData(void)
{
  if (SDCardNotPresent() || !file_.fs) return;

  struct TRPacket {
    uint16_t ir;
    uint16_t sonar;
  } tr_packet;

  uint16_t header = 0x0101;
  WriteToFIFO((char *)&header, 2);
  tr_packet.ir = TRIR();
  tr_packet.sonar = TRSonar();
  WriteToFIFO((char *)&tr_packet, sizeof(struct TRPacket));
  uint16_t crc = CRCCCITT((uint8_t *)&tr_packet, sizeof(struct TRPacket));
  WriteToFIFO((char *)&crc, 2);
}

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
  // WriteToFIFO((char *)UBXTimeUTC(), sizeof(struct UBXTimeUTC));
  // uint16_t crc = CRCCCITT((uint8_t *)UBXTimeUTC(), sizeof(struct UBXTimeUTC));
  WriteToFIFO((char *)UBXTimeUTC(), sizeof(uint32_t));
  uint16_t crc = CRCCCITT((uint8_t *)UBXTimeUTC(), sizeof(uint32_t));
  WriteToFIFO((char *)&crc, 2);
}

// -----------------------------------------------------------------------------
void LogVisionData(void)
{
  if (SDCardNotPresent() || !file_.fs) return;

  union U16Bytes temp;
  temp.bytes[0] = 0xCC;
  temp.bytes[1] = 0xDD;
  WriteToFIFO((char *)temp.bytes, 2);
  WriteToFIFO((char *)FromVision(), sizeof(struct FromVision));
  temp.u16 = CRCCCITT((uint8_t *)FromVision(), sizeof(struct FromVision));
  WriteToFIFO((char *)temp.bytes, 2);
}

// -----------------------------------------------------------------------------
void ProcessLogging(void)
{
  if (SDCardNotPresent())
  {
    logging_active_ = 0;
    return;
  }

  // if ((FlightCtrlState() & FC_STATE_BIT_MOTORS_RUNNING) && !logging_active_)
  //   OpenLogFile(0);

  // if (!(FlightCtrlState() & FC_STATE_BIT_MOTORS_RUNNING) && logging_active_)
  //   CloseLogFile();

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
      UARTPrintf("logging: f_open returned error code: 0x%02X", file_status_);
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
    UARTPrintf("logging: f_close returned error code: 0x%02X", file_status_);
    // GreenLEDOff();
    RedLEDOff();
  }
}

// -----------------------------------------------------------------------------
void LogWrite(char * buffer, uint32_t length)
{
  if (SDCardNotPresent() || !file_.fs) return;
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
