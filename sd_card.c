#include "sd_card.h"

#include "91x_lib.h"
#include "diskio.h"  // from libfatfs
#include "spi_master.h"
#include "timing.h"
#include "uart.h"


// =============================================================================
// Private data:

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

static DSTATUS status_ = STA_NODISK | STA_NOINIT;
static BYTE card_type_;


// =============================================================================
// Private function declarations:

static void CSPinHigh(void);
static void CSPinLow(void);
static inline void ClearRxFIFO(void);
static void Tx(const BYTE* buff, UINT bc);
static void Rx(BYTE *buff, UINT bc);
static uint32_t WaitForSDCard(void);
static void Deselect(void);
static uint32_t Select(void);
static uint32_t RxDataBlock(BYTE *buff, UINT btr);
static uint32_t TxDataBlock(const BYTE *buff, BYTE token);
static BYTE TxCommand( BYTE cmd, DWORD arg);


// =============================================================================
// Public functions:

void SDCardInit(void)
{
  SCU_APBPeriphClockConfig(__GPIO5, ENABLE);

  GPIO_InitTypeDef gpio_init;

  // Configure SD_SWITCH at pin GPIO5.3 as an external irq 11
  gpio_init.GPIO_Direction = GPIO_PinInput;
  gpio_init.GPIO_Pin = GPIO_Pin_3;
  gpio_init.GPIO_Type = GPIO_Type_PushPull;
  gpio_init.GPIO_IPInputConnected = GPIO_IPInputConnected_Disable;
  gpio_init.GPIO_Alternate = GPIO_InputAlt1;
  GPIO_Init(GPIO5, &gpio_init);

  // Configure P5.4 -> SD-CS as an output pin.
  gpio_init.GPIO_Direction = GPIO_PinOutput;
  gpio_init.GPIO_Pin = GPIO_Pin_4;
  gpio_init.GPIO_Alternate = GPIO_OutputAlt1;
  GPIO_Init (GPIO5, &gpio_init);

  GPIO_WriteBit(GPIO5, GPIO_Pin_4, Bit_SET);
}


// =============================================================================
// Disk I/O functions required for FatFs (declared in diskio.h and ff.h):

// This function returns the card status.
DSTATUS disk_status(BYTE drive_number)
{
  // NaviCtrl only has one card slot, so there can be only drive 0.
  if (drive_number == 0) return status_;
  return STA_NODISK | STA_NOINIT;
}

// -----------------------------------------------------------------------------
static inline uint32_t SDCardPresent(void)
{
  return !(GPIO_ReadBit(GPIO5, GPIO_Pin_3));
}

// -----------------------------------------------------------------------------
// This function initialize the SD card.
DSTATUS disk_initialize(BYTE drive_number)
{
  // NaviCtrl only has one card slot, so there can be only drive 0.
  if (drive_number != 0) return STA_NODISK | STA_NOINIT;
  if (!SDCardPresent())
  {
    UARTPrintf("sd_card: No SD card present");
    status_ = STA_NODISK | STA_NOINIT;
    return status_;
  }

  // Make sure that the SD card is deselected.
  CSPinHigh();

  // Initiate at least 74 clock cycles at 100 - 400 kHz to initialize the card.
  SPIMasterSetBaud(1000000L);
  for (uint32_t i = 10; --i; )
  {
    while (!SSP_GetFlagStatus(SSP1, SSP_FLAG_TxFifoNotFull)) continue;
    SSP_SendData(SSP1, 0xFF);
  }
  SPIMasterWaitUntilCompletion(1);  // Should require <= 0.8 ms to complete
  ClearRxFIFO();
  SPIMasterSetBaud(4000000L);

  // First assume an unidentified card type.
  card_type_ = 0;
  status_ = STA_NOINIT;

  // Request idle (reset) state.
  if (TxCommand(SD_CMD0, 0) == R1_IDLE)
  {
    // Request voltage range (only SD v2 or greater will respond).
    if (TxCommand(SD_CMD8, 0x1AA) == R1_IDLE)
    {
      BYTE rx_buffer[4];
      // Get the OCR (trailing 32-bits of the R7 response).
      Rx(rx_buffer, 4);
      // Check for a valid response (indicates SD v2 or greater).
      if (rx_buffer[2] == 0x01 && rx_buffer[3] == 0xAA)
      {
        // Wait for the card to leave idle state.
        uint32_t timer = 1000;  // Wait up to a second
        while (timer--)
        {
          if (TxCommand(SD_ACMD41, 0x40000000) == 0x00) break;
          Wait(1);
        }
        if ((timer != 0) && (TxCommand(SD_CMD58, 0) == 0))
        {
          // Check the HCS bit in the OCR, if set, card is SDHC/SDXC.
          Rx(rx_buffer, 4);
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
      if ((TxCommand(SD_ACMD41, 0) | R1_IDLE) == R1_IDLE)
      {
        card_type_ = CT_SD1;  // SD v1
        command = SD_ACMD41;
      }
      else
      {
        card_type_ = CT_MMC;  // MMC v3
        command = SD_CMD1;
      }
        // Wait for the card to leave idle state.
      uint32_t timer = 1000;  // Wait up to a second
      while (timer--)
      {
        if (TxCommand(command, 0) == 0) break;
        Wait(1);
      }
      if (timer == 0)
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

  return status_;
}

// -----------------------------------------------------------------------------
// This function reads sector(s) from the SD card.
DRESULT disk_read(BYTE drive_number, BYTE *buff, DWORD sector, UINT count)
{
  BYTE cmd;

  if (disk_status(drive_number) & STA_NOINIT) return RES_NOTRDY;
  if (!(card_type_ & CT_BLOCK)) sector *= 512;  /* Convert LBA to byte address if needed */

  cmd = count > 1 ? SD_CMD18 : SD_CMD17;      /*  READ_MULTIPLE_BLOCK : READ_SINGLE_BLOCK */
  if (TxCommand(cmd, sector) == 0) {
    do {
      if (!RxDataBlock(buff, 512)) break;
      buff += 512;
    } while (--count);
    if (cmd == SD_CMD18) TxCommand(SD_CMD12, 0); /* STOP_TRANSMISSION */
  }
  Deselect();

  return count ? RES_ERROR : RES_OK;
}

// -----------------------------------------------------------------------------
// This function writes sector(s) to the SD card.
DRESULT disk_write(BYTE drive_number, const BYTE *buff, DWORD sector, UINT count)
{
  if (disk_status(drive_number) & STA_NOINIT) return RES_NOTRDY;
  if (!(card_type_ & CT_BLOCK)) sector *= 512;  /* Convert LBA to byte address if needed */

  if (count == 1) { /* Single block write */
    if ((TxCommand(SD_CMD24, sector) == 0)  /* WRITE_BLOCK */
      && TxDataBlock(buff, 0xFE))
      count = 0;
  }
  else {        /* Multiple block write */
    if (card_type_ & CT_SDC) TxCommand(SD_ACMD23, count);
    if (TxCommand(SD_CMD25, sector) == 0) { /* WRITE_MULTIPLE_BLOCK */
      do {
        if (!TxDataBlock(buff, 0xFC)) break;
        buff += 512;
      } while (--count);
      if (!TxDataBlock(0, 0xFD)) /* STOP_TRAN token */
        count = 1;
    }
  }
  Deselect();

  return count ? RES_ERROR : RES_OK;
}
// -----------------------------------------------------------------------------
// This function controls miscellaneous functions other than generic read/write.
DRESULT disk_ioctl(BYTE drive_number, BYTE ctrl, void *buff)
{
  DRESULT res;
  BYTE n, csd[16];
  DWORD cs;


  if (disk_status(drive_number) & STA_NOINIT) return RES_NOTRDY; /* Check if card is in the socket */

  res = RES_ERROR;
  switch (ctrl) {
    case CTRL_SYNC :    /* Make sure that no pending write process */
      if (Select()) res = RES_OK;
      break;

    case GET_SECTOR_COUNT : /* Get number of sectors on the disk (DWORD) */
      if ((TxCommand(SD_CMD9, 0) == 0) && RxDataBlock(csd, 16)) {
        if ((csd[0] >> 6) == 1) { /* SDC ver 2.00 */
          cs = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
          *(DWORD*)buff = cs << 10;
        } else {          /* SDC ver 1.XX or MMC */
          n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
          cs = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
          *(DWORD*)buff = cs << (n - 9);
        }
        res = RES_OK;
      }
      break;

    case GET_BLOCK_SIZE : /* Get erase block size in unit of sector (DWORD) */
      *(DWORD*)buff = 128;
      res = RES_OK;
      break;

    default:
      res = RES_PARERR;
  }

  Deselect();

  return res;
}


// =============================================================================
// Private functions:

static void CSPinHigh(void)
{
  GPIO_WriteBit(GPIO5, GPIO_Pin_4 , Bit_SET);
}

// -----------------------------------------------------------------------------
static void CSPinLow(void)
{
  GPIO_WriteBit(GPIO5, GPIO_Pin_4 , Bit_RESET);
}

// -----------------------------------------------------------------------------
static inline void ClearRxFIFO(void)
{
  // Read bytes until the Rx FIFO is empty.
  if (SSP_GetFlagStatus(SSP1, SSP_FLAG_RxFifoNotEmpty)) SSP_ReceiveData(SSP1);
}

// -----------------------------------------------------------------------------
static void Tx(const BYTE* buff, UINT bc)
{
  do {
    SSP_SendData(SSP1, *buff++);
    while(SSP_GetFlagStatus(SSP1, SSP_FLAG_RxFifoNotEmpty)==RESET);
    SSP_ReceiveData(SSP1);
  } while (--bc);
}

// -----------------------------------------------------------------------------
// Receive bytes from the card
static void Rx(BYTE *buff, UINT bc)
{
  do {
    SSP_SendData(SSP1, 0xFF);
    while(SSP_GetFlagStatus(SSP1, SSP_FLAG_RxFifoNotEmpty)==RESET);
    *buff++ = SSP_ReceiveData(SSP1);
  } while (--bc);
}

// -----------------------------------------------------------------------------
// Wait for card ready
static uint32_t WaitForSDCard(void)  /* 1:OK, 0:Timeout */
{
  BYTE d;

  uint32_t timeout = GetTimestampMillisFromNow(500);
  while (!TimestampInPast(timeout))
  {
    Rx(&d, 1);
    if (d == 0xFF) break;
    Wait(1);  // dly_us(100);
  }

  return !TimestampInPast(timeout);
}

// -----------------------------------------------------------------------------
// Deselect the card and release SPI bus
static void Deselect(void)
{
  BYTE d;

  CSPinHigh();       /* Set CS# high */
  Rx(&d, 1);  /* Dummy clock (force DO hi-z for multiple slave SPI) */
}

// -----------------------------------------------------------------------------
// Select the card and wait for ready
static uint32_t Select(void)  /* 1:OK, 0:Timeout */
{
  BYTE d;

  CSPinLow();       /* Set CS# low */
  Rx(&d, 1);  /* Dummy clock (force DO enabled) */
  if (WaitForSDCard()) return 1; /* Wait for card ready */

  Deselect();
  return 0;     /* Failed */
}

// -----------------------------------------------------------------------------
// Receive a data packet from the card
static uint32_t RxDataBlock(BYTE *buff, UINT btr)
{
  BYTE d[2];
  UINT tmr;


  for (tmr = 1000; tmr; tmr--) {  /* Wait for data packet in timeout of 100ms */
    Rx(d, 1);
    if (d[0] != 0xFF) break;
    Wait(1);  // dly_us(100);
  }
  if (d[0] != 0xFE) return 0;   /* If not valid data token, return with error */

  Rx(buff, btr);      /* Receive the data block into buffer */
  Rx(d, 2);         /* Discard CRC */

  return 1;           /* Return with success */
}

// -----------------------------------------------------------------------------
// Send a data packet to the card
static uint32_t TxDataBlock(const BYTE *buff, BYTE token)
{
  BYTE d[2];


  if (!WaitForSDCard()) return 0;

  d[0] = token;
  Tx(d, 1);       /* Xmit a token */
  if (token != 0xFD) {    /* Is it data token? */
    Tx(buff, 512);  /* Xmit the 512 byte data block to MMC */
    Rx(d, 2);     /* Xmit dummy CRC (0xFF,0xFF) */
    Rx(d, 1);     /* Receive data response */
    if ((d[0] & 0x1F) != 0x05)  /* If not accepted, return with error */
      return 0;
  }

  return 1;
}

// -----------------------------------------------------------------------------
// Send a command packet to the card
static BYTE TxCommand( BYTE cmd, DWORD arg)
{
  BYTE n, d, buf[6];


  if (cmd & 0x80) { /* SD_ACMD<n> is the command sequense of SD_CMD55-SD_CMD<n> */
    cmd &= 0x7F;
    n = TxCommand(SD_CMD55, 0);
    if (n > 1) return n;
  }

  /* Select the card and wait for ready except to stop multiple block read */
  if (cmd != SD_CMD12) {
    Deselect();
    if (!Select()) return 0xFF;
  }

  /* Send a command packet */
  buf[0] = 0x40 | cmd;      /* Start + Command index */
  buf[1] = (BYTE)(arg >> 24);   /* arg[31..24] */
  buf[2] = (BYTE)(arg >> 16);   /* arg[23..16] */
  buf[3] = (BYTE)(arg >> 8);    /* arg[15..8] */
  buf[4] = (BYTE)arg;       /* arg[7..0] */
  n = 0x01;           /* Dummy CRC + Stop */
  if (cmd == SD_CMD0) n = 0x95;    /* (valid CRC for SD_CMD0(0)) */
  if (cmd == SD_CMD8) n = 0x87;    /* (valid CRC for SD_CMD8(0x1AA)) */
  buf[5] = n;
  Tx(buf, 6);

  /* Receive command response */
  if (cmd == SD_CMD12) Rx(&d, 1);  /* Skip a stuff byte when stop reading */
  n = 10;               /* Wait for a valid response in timeout of 10 attempts */
  do
    Rx(&d, 1);
  while ((d & 0x80) && --n);

  return d;     /* Return with the response value */
}
