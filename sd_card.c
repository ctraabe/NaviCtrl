// This file adapts the FatFS generic SD/MMC card access example to the STR91x.

// This file implements disk I/O functions that are required for FatFs. These
// functions are declared in diskio.h and ff.h, so the names cannot be changed.

#include "sd_card.h"

#include "91x_lib.h"
#include "diskio.h"  // from libfatfs
#include "ff.h"  // from libfatfs
#include "irq_priority.h"
#include "timing.h"
#include "uart.h"
// TODO: remove
#include "led.h"


// =============================================================================
// Private data:

#define CLOCK_DIVIDER_SPI1 (2)  // Must be even

// MMC/SD SPI mode commands
enum SDCommands {
  SD_CMD0 = 0,  // GO_IDLE_STATE
  SD_CMD1 = 1,  // SEND_OP_COND
  SD_ACMD41 = 0x80+41,  // SEND_OP_COND (SDC)
  SD_CMD8 = 8,  // SEND_IF_COND
  SD_CMD9 = 9,  // SEND_CSD
  SD_CMD10 = 10,  // SEND_CID
  SD_CMD12 = 12,  // STOP_TRANSMISSION
  SD_CMD13 = 13,  // SEND_STATUS
  SD_ACMD13 = 0x80+13,  // SD_STATUS (SDC)
  SD_CMD16 = 16,  // SET_BLOCKLEN
  SD_CMD17 = 17,  // READ_SINGLE_BLOCK
  SD_CMD18 = 18,  // READ_MULTIPLE_BLOCK
  SD_CMD23 = 23,  // SET_BLOCK_COUNT
  SD_ACMD23 = 0x80+23,  // SET_WR_BLK_ERASE_COUNT (SDC)
  SD_CMD24 = 24,  // WRITE_BLOCK
  SD_CMD25 = 25,  // WRITE_MULTIPLE_BLOCK
  SD_CMD32 = 32,  // ERASE_ER_BLK_START
  SD_CMD33 = 33,  // ERASE_ER_BLK_END
  SD_CMD38 = 38,  // ERASE
  SD_CMD55 = 55,  // APP_CMD
  SD_CMD58 = 58,  // READ_OCR
};

enum R1ResponseBits {
  R1_IDLE = 1<<0,
  R1_ERASE_RESET = 1<<1,
  R1_ILLEGAL_COMMAND = 1<<2,
  R1_COMMAND_CRC_ERROR = 1<<3,
  R1_ERASE_SEQUENCE_ERROR = 1<<4,
  R1_ADDRESS_ERROR = 1<<5,
  R1_PARAMETER_ERROR = 1<<6,
};

static volatile UINT bytes_remaining_ = 0;
static volatile UINT rx_bytes_remaining_ = 0, tx_bytes_remaining_ = 0;
static volatile BYTE * volatile rx_buffer_ = 0;
static const BYTE * volatile tx_buffer_ = 0;
static DSTATUS status_ = STA_NODISK | STA_NOINIT;
static BYTE card_type_ = 0;
static FATFS fat_fs_ = { 0 };
static FRESULT fs_status_ = FR_INVALID_DRIVE;


// =============================================================================
// Private function declarations:

static inline void CSPinHigh(void);
static inline void CSPinLow(void);
static inline void Deselect(void);
static uint32_t Select(void);
static void RxBuffer(BYTE * buffer, UINT length);
static BYTE RxByte(void);
static void TxBuffer(const BYTE * buffer, UINT length);
static void TxByte(BYTE byte);
static uint32_t RxDataBlock(BYTE * buffer, UINT length);
static uint32_t TxDataBlock(const BYTE * buffer, BYTE token);
static BYTE TxCommand(BYTE command, DWORD argument);
static void SPIStart(size_t exchange_length, BYTE * rx_buffer, size_t rx_length,
  const BYTE * tx_buffer, size_t tx_length);
static void SetBaud(uint32_t baud_rate);
static uint32_t WaitForSDCard(void);
static uint32_t WaitForSPI(uint32_t time_limit_ms);


// =============================================================================
// Public functions:

void SDCardInit(void)
{
  SCU_APBPeriphClockConfig(__GPIO3 ,ENABLE);
  SCU_APBPeriphClockConfig(__GPIO5, ENABLE);
  SCU_APBPeriphClockConfig(__SSP1 ,ENABLE);
  SCU_APBPeriphClockConfig(__WIU, ENABLE);

  GPIO_InitTypeDef gpio_init;

  // Configure P5.4 -> SD-CS as an output pin.
  gpio_init.GPIO_Direction = GPIO_PinOutput;
  gpio_init.GPIO_Pin = GPIO_Pin_4;
  gpio_init.GPIO_Type = GPIO_Type_PushPull;
  gpio_init.GPIO_IPInputConnected = GPIO_IPInputConnected_Disable;
  gpio_init.GPIO_Alternate = GPIO_OutputAlt1;
  GPIO_Init (GPIO5, &gpio_init);

  // Configure SD_SWITCH at pin GPIO5.3 as an external interrupt (EXTIT11)
  gpio_init.GPIO_Direction = GPIO_PinInput;
  gpio_init.GPIO_Pin = GPIO_Pin_3;
  gpio_init.GPIO_Alternate = GPIO_InputAlt1;
  GPIO_Init(GPIO5, &gpio_init);

  // Configure P3.5 <- MISO1 as an input pin.
  gpio_init.GPIO_Pin = GPIO_Pin_5;
  gpio_init.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
  GPIO_Init (GPIO3, &gpio_init);

  // Configure P3.4 -> SCK1 and P3.6 -> MOSI1 as output pins.
  gpio_init.GPIO_Direction = GPIO_PinOutput;
  gpio_init.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
  gpio_init.GPIO_Alternate = GPIO_OutputAlt2;
  GPIO_Init (GPIO3, &gpio_init);

  // Configure WIU to trigger an interrupt on a change in pin P6.0.
  WIU_InitTypeDef wiu_init;
  wiu_init.WIU_Line = WIU_Line11;  // Pin P5.3
  if (SDCardNotPresent())
    wiu_init.WIU_TriggerEdge = WIU_FallingEdge;
  else
    wiu_init.WIU_TriggerEdge = WIU_RisingEdge;
  WIU_Init(&wiu_init);

  WIU_ClearITPendingBit(WIU_Line11);
  SCU_WakeUpLineConfig(11);
  WIU_Cmd(ENABLE);

  VIC_Config(EXTIT1_ITLine, VIC_IRQ, IRQ_PRIORITY_SD_PRESENT);
  VIC_ITCmd(EXTIT1_ITLine, ENABLE);

  // Deselect the SD Card.
  CSPinHigh();

  if (SDCardNotPresent())
    UARTPrintf("logging: SD card not present");
  else
    SDCardMountFS();
}

// -----------------------------------------------------------------------------
void SDCardMountFS(void)
{
  fs_status_ = f_mount(&fat_fs_, "", 1);
}

// -----------------------------------------------------------------------------
void SDCardUnmountFS(void)
{
  fs_status_ = f_mount(NULL, "", 0);
}

// -----------------------------------------------------------------------------
uint32_t SDCardFSMounted(void)
{
  return !SDCardNotPresent() && (fs_status_ == FR_OK);
}

// -----------------------------------------------------------------------------
uint32_t SDCardNotPresent(void)
{
  return GPIO_ReadBit(GPIO5, GPIO_Pin_3);
}


// =============================================================================
// Disk I/O functions required for FatFs (declared in diskio.h and ff.h):

// This function returns the card status.
DSTATUS disk_status(BYTE drive_number)
{
  // UARTPrintf("sd_card: disk_status(%u)", drive_number);

  // NaviCtrl only has one card slot, so there can be only drive 0.
  if (drive_number == 0) return status_;
  return STA_NODISK | STA_NOINIT;
}

// -----------------------------------------------------------------------------
// This function initialize the SD card.
DSTATUS disk_initialize(BYTE drive_number)
{
  // UARTPrintf("sd_card: disk_initialize(%u)", drive_number);

  // NaviCtrl only has one card slot, so there can be only drive 0.
  if (drive_number != 0) return STA_NODISK | STA_NOINIT;
  if (SDCardNotPresent())
  {
    status_ = STA_NODISK | STA_NOINIT;
    return status_;
  }

  // Make sure that the SD card is deselected.
  CSPinHigh();

  // Initiate at least 74 clock cycles at 100 - 400 kHz to initialize the card.
  SetBaud(400000L);
  SPIStart(10, 0, 0, 0, 0);
  WaitForSPI(10);  // Should require 0.2 ms to complete
  while (SSP_GetFlagStatus(SSP1, SSP_FLAG_RxFifoNotEmpty))
    SSP_ReceiveData(SSP1);

  // Reinitialize SPI at 1 MHz.
  SetBaud(1000000L);

  // First assume an unidentified card type.
  card_type_ = 0;
  status_ = STA_NOINIT;

  // Request idle (reset) state.
  if (TxCommand(SD_CMD0, 0x00000000) == R1_IDLE)
  {
    // Request voltage range (only SD v2 or greater will respond).
    if (TxCommand(SD_CMD8, 0x000001AA) == R1_IDLE)
    {
      BYTE rx_buffer[4];
      // Get the OCR (trailing 32-bits of the R7 response).
      RxBuffer(rx_buffer, 4);
      // Check for a valid response (indicates SD v2 or greater).
      if (rx_buffer[2] == 0x01 && rx_buffer[3] == 0xAA)
      {
        // Wait up to a second for the card to leave idle state.
        UINT ms_timer = 1000;
        while ((TxCommand(SD_ACMD41, 0x40000000) != 0x00) && --ms_timer)
          Wait(1);
        if ((ms_timer != 0) && (TxCommand(SD_CMD58, 0x00000000) == 0x00))
        {
          // Check the HCS bit in the OCR, if set, card is SDHC/SDXC.
          RxBuffer(rx_buffer, 4);
          card_type_ = (rx_buffer[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
        }
        else
        {
          UARTPrintf("sd_card: SD card (v2) failed to leave idle state");
        }
      }
      else
      {
        UARTPrintf("sd_card: Mismatch in CMD8 R7 response");
      }
    }
    else
    {  // SD v1 or MMC v3
      BYTE command;
      // Send an initialization command (only SD cards will respond).
      if ((TxCommand(SD_ACMD41, 0x00000000) | R1_IDLE) == R1_IDLE)
      {
        card_type_ = CT_SD1;  // SD v1
        command = SD_ACMD41;
      }
      else
      {
        card_type_ = CT_MMC;  // MMC v3
        command = SD_CMD1;
      }
        // Wait up to a second for the card to leave idle state.
      UINT ms_timer = 1000;
      while ((TxCommand(command, 0x00000000) != 0) && --ms_timer) Wait(1);
      if (ms_timer == 0)
      {
        UARTPrintf("sd_card: SD card failed to leave idle state");
        card_type_ = 0;
      }
    }

    // Request the block length to be set to 512 bytes.
    if (!(card_type_ & CT_BLOCK) && (TxCommand(SD_CMD16, 512) != 0x00))
    {
      UARTPrintf("sd_card: request for 512-byte block length was not honored");
      card_type_ = 0;
    }
  }
  else
  {
    UARTPrintf("sd_card: SD card failed to respond");
  }

  if (card_type_ != 0)
  {
    UARTPrintf("sd_card: SD card type %X detected and initialized");
    status_ = 0;
  }
  else
  {
    status_ |= STA_NOINIT;
  }

  Deselect();
  // UARTPrintf("sd_card: disk_initialize(): 0x%02X", status_);
  return status_;
}

// -----------------------------------------------------------------------------
// This function reads sector(s) from the SD card.
DRESULT disk_read(BYTE drive_number, BYTE *buffer, DWORD sector, UINT count)
{
  if ((status_ & STA_NOINIT) || (drive_number != 0)) return RES_NOTRDY;

  // Cards that are not high capacity should be byte addressed.
  if (!(card_type_ & CT_BLOCK)) sector *= 512;

  // Choose the single-byte or multiple-byte read command.
  BYTE command = count > 1 ? SD_CMD18 : SD_CMD17;

  if (TxCommand(command, sector) == 0)
  {
    do
    {
      if (RxDataBlock(buffer, 512)) break;
      buffer += 512;
    } while (--count);

    if (command == SD_CMD18) TxCommand(SD_CMD12, 0);  // Stop transmission
  }

  Deselect();
  return count ? RES_ERROR : RES_OK;
}

// -----------------------------------------------------------------------------
// This function writes sector(s) to the SD card.
DRESULT disk_write(BYTE drive_number, const BYTE *buffer, DWORD sector,
  UINT count)
{
  if ((status_ & STA_NOINIT) || (drive_number != 0)) return RES_NOTRDY;

  // Cards that are not high capacity should be byte addressed.
  if (!(card_type_ & CT_BLOCK)) sector *= 512;

  if (count == 1)
  {
    if ((TxCommand(SD_CMD24, sector) == 0) && !TxDataBlock(buffer, 0xFE))
      count = 0;
  }
  else
  {  // Multiple block write
    if (card_type_ & CT_SDC) TxCommand(SD_ACMD23, count);
    if (TxCommand(SD_CMD25, sector) == 0)
    {
      do {
        if (TxDataBlock(buffer, 0xFC)) break;
        buffer += 512;
      } while (--count);

      // Check for the "stop transmission" token.
      if (TxDataBlock(0, 0xFD)) count = 1;
    }
  }

  Deselect();
  return count ? RES_ERROR : RES_OK;
}

// -----------------------------------------------------------------------------
// This function controls miscellaneous functions other than generic read/write.
DRESULT disk_ioctl(BYTE drive_number, BYTE ctrl, void *buffer)
{
  if ((status_ & STA_NOINIT) || (drive_number != 0)) return RES_NOTRDY;

  DWORD cs;
  BYTE csd[16];
  DRESULT response = RES_ERROR;
  switch (ctrl)
  {
    case CTRL_SYNC:  // Make sure that there is no pending write process
      if (!Select()) response = RES_OK;
      break;

    case GET_SECTOR_COUNT:  // Get number of sectors on the disk (DWORD)
      if ((TxCommand(SD_CMD9, 0) == 0) && !RxDataBlock(csd, 16))
      {
        if ((csd[0] >> 6) == 1)
        {  // SDC v2
          cs = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
          *(DWORD*)buffer = cs << 10;
        }
        else
        {  // SDC v1 or MMC
          BYTE n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1)
            + 2;
          cs = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10)
            + 1;
          *(DWORD*)buffer = cs << (n - 9);
        }
        response = RES_OK;
      }
      break;

    case GET_BLOCK_SIZE:  // Get erase block size in unit of sector (DWORD)
      *(DWORD*)buffer = 128;
      response = RES_OK;
      break;

    default:
      response = RES_PARERR;
  }

  Deselect();
  return response;
}


// =============================================================================
// Private functions:

static inline void CSPinHigh(void)
{
  GPIO_WriteBit(GPIO5, GPIO_Pin_4, Bit_SET);
}

// -----------------------------------------------------------------------------
static inline void CSPinLow(void)
{
  GPIO_WriteBit(GPIO5, GPIO_Pin_4, Bit_RESET);
}

// -----------------------------------------------------------------------------
// Deselect the card and release SPI bus
static inline void Deselect(void)
{
  CSPinHigh();
  RxByte();  // Run the clock for one frame
}

// -----------------------------------------------------------------------------
// Select the card and wait for ready
static uint32_t Select(void)  /* 1:OK, 0:Timeout */
{
  CSPinLow();
  RxByte();  // Run the clock for one frame
  if (!WaitForSDCard()) return 0;  // Success

  Deselect();
  return 1;  // Failure
}

// -----------------------------------------------------------------------------
// Receive bytes from the card
static void RxBuffer(BYTE * buffer, UINT length)
{
  SPIStart(length, buffer, length, 0, 0);
  WaitForSPI(1000);
  while (SSP_GetFlagStatus(SSP1, SSP_FLAG_RxFifoNotEmpty))
    *rx_buffer_++ = SSP_ReceiveData(SSP1);
}

// -----------------------------------------------------------------------------
static BYTE RxByte(void)
{
  SSP_SendData(SSP1, 0xFF);
  while (!SSP_GetFlagStatus(SSP1, SSP_FLAG_RxFifoNotEmpty)) continue;
  return SSP_ReceiveData(SSP1);
}

// -----------------------------------------------------------------------------
static void TxBuffer(const BYTE * buffer, UINT length)
{
  SPIStart(length, 0, 0, buffer, length);
  WaitForSPI(1000);
  while (SSP_GetFlagStatus(SSP1, SSP_FLAG_RxFifoNotEmpty))
    SSP_ReceiveData(SSP1);
}

// -----------------------------------------------------------------------------
static void TxByte(BYTE byte)
{
  SSP_SendData(SSP1, byte);
  while (!SSP_GetFlagStatus(SSP1, SSP_FLAG_RxFifoNotEmpty)) continue;
  SSP_ReceiveData(SSP1);
}

// -----------------------------------------------------------------------------
// Receive a data packet from the card.
static uint32_t RxDataBlock(BYTE * buffer, UINT length)
{
  // Wait for the leading data token (DO is held high until then).
  BYTE temp;
  for (UINT timer = 1000; timer; --timer)
  {
    temp = RxByte();
    if (temp != 0xFF) break;
    MicroWait(100);
  }
  if (temp != 0xFE) return 1;  // Received byte differs from the expected token

  RxBuffer(buffer, length);
  RxByte();  // Discard the first CRC byte
  RxByte();  // Discard the second CRC byte

  return 0;  // Success
}

// -----------------------------------------------------------------------------
// Send a data packet to the card.
static uint32_t TxDataBlock(const BYTE * buffer, BYTE token)
{
  if (WaitForSDCard()) return 1;  // Timed out

  TxByte(token);  // Transmit the token that leads the data block

  // If this is a data token, then follow up with the data block.
  if (token != 0xFD)
  {
    TxBuffer(buffer, 512);
    RxByte();  // Just transmit 0xFF for the first CRC byte (ignored)
    RxByte();  // Just transmit 0xFF for the second CRC byte (ignored)
    if ((RxByte() & 0x1F) != 0x05) return 1;  // Data not accepted
  }

  return 0;  // Success
}

// -----------------------------------------------------------------------------
// Send a command packet to the card
static BYTE TxCommand(BYTE command, DWORD argument)
{
  // SD_ACMDxx must be preceded by CMD55
  if (command & 0x80)
  {
    command &= 0x7F;
    BYTE response = TxCommand(SD_CMD55, 0);
    if (response > 0x01) return response;
  }

  // Select the card and wait for ready (except to stop a multiple block read)
  if (command != SD_CMD12)
  {
    Deselect();
    if (Select()) return 0xFF;
  }

  // Send the command packet
  BYTE buffer[6];
  buffer[0] = 0x40 | command;  // Start + command index
  buffer[1] = (BYTE)(argument >> 24);
  buffer[2] = (BYTE)(argument >> 16);
  buffer[3] = (BYTE)(argument >> 8);
  buffer[4] = (BYTE)argument;
  // Note that the CRC is ignored for all commands except CMD0 and CMD8, but
  // must contain a trailing 1. Just send the CRC for CMD0 unless CMD8.
  buffer[5] = 0x95;  // valid CRC for CMD0+0x00000000
  if (command == SD_CMD8) buffer[5] = 0x87;  // valid CRC for CMD8+0x000001AA
  TxBuffer(buffer, 6);

  // Receive the response
  BYTE response;
  if (command == SD_CMD12) RxByte();  // Skip one byte of junk
  UINT timer = 10;
  do
  {
    response = RxByte();
  } while ((response & 0x80) && --timer);

  return response;
}

// -----------------------------------------------------------------------------
// Start an SPI exchange. The number of bytes to be exchanged is specified by
// exchange_lenght. These bytes may or may not be dummy bytes. The number of
// (non-dummy) bytes to be received is specified by rx_length. The number of
// (non-dummy) bytes to be transmitted is specified by tx_length. Non-dummy
// bytes are always received/transmitted first.
static void SPIStart(size_t exchange_length, BYTE * rx_buffer, size_t rx_length,
  const BYTE * tx_buffer, size_t tx_length)
{
  // First, do some sanity checking.
  if (exchange_length < rx_length) exchange_length = rx_length;
  if (exchange_length < tx_length) exchange_length = tx_length;

  // Set the static variables.
  bytes_remaining_ = exchange_length;
  rx_buffer_ = rx_buffer;
  tx_buffer_ = tx_buffer;
  rx_bytes_remaining_ = rx_length;
  tx_bytes_remaining_ = tx_length;

  // Enable the TX FIFO interrupt.
  SSP_ITConfig(SSP1, SSP_IT_TxFifo, ENABLE);
}

// -----------------------------------------------------------------------------
// Note that NaviCtrl does not seem to be capable of baud faster than 0.5 MHz @
// 48 MHz master clock and 1 MHz @ 96 MHz master clock.
static void SetBaud(uint32_t baud_rate)
{
  uint32_t baud_rate_clock = SCU_GetMCLKFreqValue() * 1000;
  if (!(SCU->CLKCNTR & SCU_BRCLK_Div1)) baud_rate_clock /= 2;
  uint32_t prescaler = baud_rate_clock / (CLOCK_DIVIDER_SPI1 * baud_rate) - 1;

  SSP_InitTypeDef ssp_init;

  SSP_StructInit(&ssp_init);
  ssp_init.SSP_ClockRate = CLOCK_DIVIDER_SPI1;
  ssp_init.SSP_ClockPrescaler = (uint8_t)prescaler;
  SSP_DeInit(SSP1);
  SSP_Init(SSP1, &ssp_init);
  SSP_Cmd(SSP1, ENABLE);

  SSP_ITConfig(SSP1, SSP_IT_RxFifo, ENABLE);
  VIC_Config(SSP1_ITLine, VIC_IRQ, IRQ_PRIORITY_SSP1);
  VIC_ITCmd(SSP1_ITLine, ENABLE);
}

// -----------------------------------------------------------------------------
// Wait up to 500 ms for the SD card to exit the busy state.
static uint32_t WaitForSDCard(void)
{
  uint32_t timeout = GetTimestampMillisFromNow(500);
  while ((RxByte() != 0xFF) && !TimestampInPast(timeout)) MicroWait(100);
  return TimestampInPast(timeout);
}

// -----------------------------------------------------------------------------
// Wait up to time_limit_ms for the SPI exchange to finish.
static uint32_t WaitForSPI(uint32_t time_limit_ms)
{
  uint32_t timeout = GetTimestampMillisFromNow(time_limit_ms);
  while (SSP_GetFlagStatus(SSP1, SSP_FLAG_Busy) && !TimestampInPast(timeout))
    continue;
  return TimestampInPast(timeout);
}

// -----------------------------------------------------------------------------
void SDSPIHandler(void)
{
  while (SSP_GetFlagStatus(SSP1, SSP_FLAG_RxFifoNotEmpty))
  {
    if (rx_bytes_remaining_ != 0 && rx_buffer_)
    {
      *rx_buffer_++ = SSP_ReceiveData(SSP1);
      --rx_bytes_remaining_;
    }
    else
    {
      SSP_ReceiveData(SSP1);
    }
  }

  while (SSP_GetFlagStatus(SSP1, SSP_FLAG_TxFifoNotFull) && bytes_remaining_)
  {
    if (tx_bytes_remaining_ != 0 && tx_buffer_)
    {
      SSP_SendData(SSP1, *tx_buffer_++);
      --tx_bytes_remaining_;
    }
    else
    {
      SSP_SendData(SSP1, 0xFF);
    }
    --bytes_remaining_;
  }

  if (bytes_remaining_ == 0) SSP_ITConfig(SSP1, SSP_IT_TxFifo, DISABLE);
}

// -----------------------------------------------------------------------------
// This interrupt occurs when an SD card is inserted.
void SDCardPresentHandler(void)
{
  WIU_ClearITPendingBit(WIU_Line11);

  // Configure WIU to trigger an interrupt on a change in pin P6.0.
  WIU_InitTypeDef wiu_init;
  wiu_init.WIU_Line = WIU_Line11;  // Pin P5.3
  if (SDCardNotPresent())
  {
    wiu_init.WIU_TriggerEdge = WIU_FallingEdge;
    SDCardUnmountFS();
  }
  else
  {
    wiu_init.WIU_TriggerEdge = WIU_RisingEdge;
    Wait(1000);
    SDCardMountFS();
  }
  WIU_Init(&wiu_init);
}
