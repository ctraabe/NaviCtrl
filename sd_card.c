#include "sd_card.h"

#include "91x_lib.h"
#include "diskio.h"  // from libfatfs
#include "spi_master.h"
#include "timing.h"


// =============================================================================
// Private data:

// MMC/SD SPI mode commands
#define CMD0  (0)  // GO_IDLE_STATE
#define CMD1  (1)  // SEND_OP_COND
#define ACMD41  (0x80+41)  // SEND_OP_COND (SDC)
#define CMD8  (8)  // SEND_IF_COND
#define CMD9  (9)  // SEND_CSD
#define CMD10 (10)  // SEND_CID
#define CMD12 (12)  // STOP_TRANSMISSION
#define CMD13 (13)  // SEND_STATUS
#define ACMD13  (0x80+13)  // SD_STATUS (SDC)
#define CMD16 (16)  // SET_BLOCKLEN
#define CMD17 (17)  // READ_SINGLE_BLOCK
#define CMD18 (18)  // READ_MULTIPLE_BLOCK
#define CMD23 (23)  // SET_BLOCK_COUNT
#define ACMD23  (0x80+23)  // SET_WR_BLK_ERASE_COUNT (SDC)
#define CMD24 (24)  // WRITE_BLOCK
#define CMD25 (25)  // WRITE_MULTIPLE_BLOCK
#define CMD32 (32)  // ERASE_ER_BLK_START
#define CMD33 (33)  // ERASE_ER_BLK_END
#define CMD38 (38)  // ERASE
#define CMD55 (55)  // APP_CMD
#define CMD58 (58)  // READ_OCR

static DSTATUS Stat = STA_NODISK | STA_NOINIT;
static BYTE CardType;


// =============================================================================
// Private function declarations:


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
static void xmit_mmc(const BYTE* buff, UINT bc)
{
  do {
    SSP_SendData(SSP1, *buff++);
    while(SSP_GetFlagStatus(SSP1, SSP_FLAG_RxFifoNotEmpty)==RESET);
    SSP_ReceiveData(SSP1);
  } while (--bc);
}

// -----------------------------------------------------------------------------
// Receive bytes from the card
static void rcvr_mmc(BYTE *buff, UINT bc)
{
  do {
    SSP_SendData(SSP1, 0xFF);
    while(SSP_GetFlagStatus(SSP1, SSP_FLAG_RxFifoNotEmpty)==RESET);
    *buff++ = SSP_ReceiveData(SSP1);
  } while (--bc);
}

// -----------------------------------------------------------------------------
// Wait for card ready
static int wait_ready(void)  /* 1:OK, 0:Timeout */
{
  BYTE d;

  uint32_t timeout = GetTimestampMillisFromNow(500);
  while (!TimestampInPast(timeout))
  {
    rcvr_mmc(&d, 1);
    if (d == 0xFF) break;
    Wait(1);  // dly_us(100);
  }

  return !TimestampInPast(timeout);
}

// -----------------------------------------------------------------------------
// Deselect the card and release SPI bus
static void deselect(void)
{
  BYTE d;

  CSPinHigh();       /* Set CS# high */
  rcvr_mmc(&d, 1);  /* Dummy clock (force DO hi-z for multiple slave SPI) */
}

// -----------------------------------------------------------------------------
// Select the card and wait for ready
static int select(void)  /* 1:OK, 0:Timeout */
{
  BYTE d;

  CSPinLow();       /* Set CS# low */
  rcvr_mmc(&d, 1);  /* Dummy clock (force DO enabled) */
  if (wait_ready()) return 1; /* Wait for card ready */

  deselect();
  return 0;     /* Failed */
}

// -----------------------------------------------------------------------------
// Receive a data packet from the card
static int rcvr_datablock(BYTE *buff, UINT btr)
{
  BYTE d[2];
  UINT tmr;


  for (tmr = 1000; tmr; tmr--) {  /* Wait for data packet in timeout of 100ms */
    rcvr_mmc(d, 1);
    if (d[0] != 0xFF) break;
    Wait(1);  // dly_us(100);
  }
  if (d[0] != 0xFE) return 0;   /* If not valid data token, return with error */

  rcvr_mmc(buff, btr);      /* Receive the data block into buffer */
  rcvr_mmc(d, 2);         /* Discard CRC */

  return 1;           /* Return with success */
}

// -----------------------------------------------------------------------------
// Send a data packet to the card
static int xmit_datablock(const BYTE *buff, BYTE token)
{
  BYTE d[2];


  if (!wait_ready()) return 0;

  d[0] = token;
  xmit_mmc(d, 1);       /* Xmit a token */
  if (token != 0xFD) {    /* Is it data token? */
    xmit_mmc(buff, 512);  /* Xmit the 512 byte data block to MMC */
    rcvr_mmc(d, 2);     /* Xmit dummy CRC (0xFF,0xFF) */
    rcvr_mmc(d, 1);     /* Receive data response */
    if ((d[0] & 0x1F) != 0x05)  /* If not accepted, return with error */
      return 0;
  }

  return 1;
}

// -----------------------------------------------------------------------------
// Send a command packet to the card
static BYTE send_cmd( BYTE cmd, DWORD arg)
{
  BYTE n, d, buf[6];


  if (cmd & 0x80) { /* ACMD<n> is the command sequense of CMD55-CMD<n> */
    cmd &= 0x7F;
    n = send_cmd(CMD55, 0);
    if (n > 1) return n;
  }

  /* Select the card and wait for ready except to stop multiple block read */
  if (cmd != CMD12) {
    deselect();
    if (!select()) return 0xFF;
  }

  /* Send a command packet */
  buf[0] = 0x40 | cmd;      /* Start + Command index */
  buf[1] = (BYTE)(arg >> 24);   /* Argument[31..24] */
  buf[2] = (BYTE)(arg >> 16);   /* Argument[23..16] */
  buf[3] = (BYTE)(arg >> 8);    /* Argument[15..8] */
  buf[4] = (BYTE)arg;       /* Argument[7..0] */
  n = 0x01;           /* Dummy CRC + Stop */
  if (cmd == CMD0) n = 0x95;    /* (valid CRC for CMD0(0)) */
  if (cmd == CMD8) n = 0x87;    /* (valid CRC for CMD8(0x1AA)) */
  buf[5] = n;
  xmit_mmc(buf, 6);

  /* Receive command response */
  if (cmd == CMD12) rcvr_mmc(&d, 1);  /* Skip a stuff byte when stop reading */
  n = 10;               /* Wait for a valid response in timeout of 10 attempts */
  do
    rcvr_mmc(&d, 1);
  while ((d & 0x80) && --n);

  return d;     /* Return with the response value */
}


// =============================================================================
// Disk I/O functions required for FatFs (declared in diskio.h and ff.h):

// This function returns the card status.
DSTATUS disk_status(BYTE drv)
{
  if (drv == 0) return Stat;
  return STA_NODISK | STA_NOINIT;
}

// -----------------------------------------------------------------------------
// This function initialize the SD card.
DSTATUS disk_initialize(BYTE drv)
{
  BYTE n, ty, cmd, buf[4];
  UINT tmr;
  DSTATUS s;

  if (drv) return RES_NOTRDY;

  Wait(10);  // dly_us(10000);      /* 10ms */
  CSPinHigh();    /* Initialize port pin tied to CS */

  for (n = 10; n; n--) rcvr_mmc(buf, 1);  /* Apply 80 dummy clocks and the card gets ready to receive command */

  ty = 0;
  if (send_cmd(CMD0, 0) == 1) {     /* Enter Idle state */
    if (send_cmd(CMD8, 0x1AA) == 1) { /* SDv2? */
      rcvr_mmc(buf, 4);             /* Get trailing return value of R7 resp */
      if (buf[2] == 0x01 && buf[3] == 0xAA) {   /* The card can work at vdd range of 2.7-3.6V */
        for (tmr = 1000; tmr; tmr--) {      /* Wait for leaving idle state (ACMD41 with HCS bit) */
          if (send_cmd(ACMD41, 1UL << 30) == 0) break;
          Wait(1);  // dly_us(1000);
        }
        if (tmr && send_cmd(CMD58, 0) == 0) { /* Check CCS bit in the OCR */
          rcvr_mmc(buf, 4);
          ty = (buf[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;  /* SDv2 */
        }
      }
    } else {              /* SDv1 or MMCv3 */
      if (send_cmd(ACMD41, 0) <= 1)   {
        ty = CT_SD1; cmd = ACMD41;  /* SDv1 */
      } else {
        ty = CT_MMC; cmd = CMD1;  /* MMCv3 */
      }
      for (tmr = 1000; tmr; tmr--) {      /* Wait for leaving idle state */
        if (send_cmd(cmd, 0) == 0) break;
        Wait(1);  // dly_us(1000);
      }
      if (!tmr || send_cmd(CMD16, 512) != 0)  /* Set R/W block length to 512 */
        ty = 0;
    }
  }
  CardType = ty;
  s = ty ? 0 : STA_NOINIT;
  Stat = s;

  deselect();

  return s;
}

// -----------------------------------------------------------------------------
// This function reads sector(s) from the SD card.
DRESULT disk_read(BYTE drv, BYTE *buff, DWORD sector, UINT count)
{
  BYTE cmd;

  if (disk_status(drv) & STA_NOINIT) return RES_NOTRDY;
  if (!(CardType & CT_BLOCK)) sector *= 512;  /* Convert LBA to byte address if needed */

  cmd = count > 1 ? CMD18 : CMD17;      /*  READ_MULTIPLE_BLOCK : READ_SINGLE_BLOCK */
  if (send_cmd(cmd, sector) == 0) {
    do {
      if (!rcvr_datablock(buff, 512)) break;
      buff += 512;
    } while (--count);
    if (cmd == CMD18) send_cmd(CMD12, 0); /* STOP_TRANSMISSION */
  }
  deselect();

  return count ? RES_ERROR : RES_OK;
}

// -----------------------------------------------------------------------------
// This function writes sector(s) to the SD card.
DRESULT disk_write(BYTE drv, const BYTE *buff, DWORD sector, UINT count)
{
  if (disk_status(drv) & STA_NOINIT) return RES_NOTRDY;
  if (!(CardType & CT_BLOCK)) sector *= 512;  /* Convert LBA to byte address if needed */

  if (count == 1) { /* Single block write */
    if ((send_cmd(CMD24, sector) == 0)  /* WRITE_BLOCK */
      && xmit_datablock(buff, 0xFE))
      count = 0;
  }
  else {        /* Multiple block write */
    if (CardType & CT_SDC) send_cmd(ACMD23, count);
    if (send_cmd(CMD25, sector) == 0) { /* WRITE_MULTIPLE_BLOCK */
      do {
        if (!xmit_datablock(buff, 0xFC)) break;
        buff += 512;
      } while (--count);
      if (!xmit_datablock(0, 0xFD)) /* STOP_TRAN token */
        count = 1;
    }
  }
  deselect();

  return count ? RES_ERROR : RES_OK;
}
// -----------------------------------------------------------------------------
// This function controls miscellaneous functions other than generic read/write.
DRESULT disk_ioctl(BYTE drv, BYTE ctrl, void *buff)
{
  DRESULT res;
  BYTE n, csd[16];
  DWORD cs;


  if (disk_status(drv) & STA_NOINIT) return RES_NOTRDY; /* Check if card is in the socket */

  res = RES_ERROR;
  switch (ctrl) {
    case CTRL_SYNC :    /* Make sure that no pending write process */
      if (select()) res = RES_OK;
      break;

    case GET_SECTOR_COUNT : /* Get number of sectors on the disk (DWORD) */
      if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
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

  deselect();

  return res;
}
