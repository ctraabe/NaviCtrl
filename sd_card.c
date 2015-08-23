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

static DSTATUS status_ = STA_NODISK | STA_NOINIT;


// =============================================================================
// Private function declarations:

static void Deselect(void);
static uint32_t SDCardPresent(void);
static uint32_t Select(void);
static uint8_t SendCommand(uint8_t cmd, uint32_t arg);
static uint32_t WaitForReady(uint32_t time_limit_ms);

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
  GPIO_Init (GPIO5, &gpio_init);
}


// =============================================================================
// Disk I/O functions required for FatFs (declared in diskio.h and ff.h):

// This function returns the card status.
DSTATUS disk_status(BYTE pdrv)
{
  if (pdrv == 0) return status_;
  return STA_NODISK | STA_NOINIT;
}

// -----------------------------------------------------------------------------
// This function initialize the SD card.
DSTATUS disk_initialize(BYTE pdrv)
{
  if (pdrv != 0) return STA_NODISK | STA_NOINIT;

  Wait(10);

  Deselect();
  // TODO: Is this necessary?
  SPIMasterStart(10, 0);
  SPIMasterWaitUntilCompletion(10);
  SPIMasterResetRxFIFO();

  return STA_NOINIT;
}

// -----------------------------------------------------------------------------
// This function reads sector(s) from the SD card.
DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
  return RES_PARERR;
}

// -----------------------------------------------------------------------------
// This function writes sector(s) to the SD card.
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
  return RES_PARERR;
}
// -----------------------------------------------------------------------------
// This function controls miscellaneous functions other than generic read/write.
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  return RES_PARERR;
}

// -----------------------------------------------------------------------------
DWORD get_fattime(void)
{
  return 0;
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
static void Deselect(void)
{
  CSPinHigh();
  // TODO: Is this necessary for single slave applications?
  SPIMasterStart(1, 0);
  SPIMasterWaitUntilCompletion(10);
  SPIMasterResetRxFIFO();
}

// -----------------------------------------------------------------------------
static uint32_t SDCardPresent(void)
{
  return !(GPIO_ReadBit(GPIO5, GPIO_Pin_3));
}

// -----------------------------------------------------------------------------
static uint32_t Select(void)
{
  CSPinLow();

  // Establish the clock to enable DO
  SPIMasterStart(1, 0);
  SPIMasterWaitUntilCompletion(10);
  SPIMasterResetRxFIFO();

  // TODO: Is this too long?
  if (!WaitForReady(500)) return 0;

  // Deselect of the card failed to respond.
  Deselect();
  return 1;
}

// -----------------------------------------------------------------------------
static uint8_t SendCommand(uint8_t cmd, uint32_t arg)
{
  uint8_t n;

  // ACMDXX begins with a sequence of CMD55 commands until the response is 0.
  if (cmd & 0x80)
  {
    cmd &= 0x7F;
    n = SendCommand(CMD55, 0);
    if (n > 1) return n;
  }

  // Select the card and wait for ready response.
  if (cmd != CMD12)
  {
    Deselect();
    if (Select()) return 0xFF;  // Card did not respond
  }

  // Send a command packet.
  uint8_t * tx_buffer = RequestSPIMasterTxBuffer();
  if (!tx_buffer) return 0xFF;  // TX buffer not ready
  tx_buffer[0] = 0x40 | cmd;  // Start + Command
  tx_buffer[1] = (uint8_t)(arg >> 24);  // Argument[31..24]
  tx_buffer[2] = (uint8_t)(arg >> 16);  // Argument[23..16]
  tx_buffer[3] = (uint8_t)(arg >> 8);  // Argument[15..8]
  tx_buffer[4] = (uint8_t)arg;  // Argument[7..0]
  n = 0x01;  // Dummy CRC + Stop
  if (cmd == CMD0) n = 0x95;  // (valid CRC for CMD0(0))
  if (cmd == CMD8) n = 0x87;  // (valid CRC for CMD8(0x1AA))
  tx_buffer[5] = n;
  SPIMasterStart(0, 6);

  // Skip one received byte following a "stop transmission" command.
  if (cmd == CMD12)
  {
    SPIMasterStart(1, 0);
    SPIMasterWaitUntilCompletion(10);
    SPIMasterResetRxFIFO();
  }

  // Get the response to the command.
  uint8_t response = 0;
  for (uint32_t i = 10; --i && (response & 0x80); )
  {
    SPIMasterStart(1, 0);
    SPIMasterWaitUntilCompletion(10);
    while(!SPIMasterGetByte(&response));
  }

  return response;
}

// -----------------------------------------------------------------------------
static uint32_t WaitForReady(uint32_t time_limit_ms)
{
  uint32_t timeout = GetTimestampMillisFromNow(time_limit_ms);
  uint8_t response = 0;
  while ((response != 0xFF) && !TimestampInPast(timeout))
  {
    SPIMasterStart(1, 0);
    SPIMasterWaitUntilCompletion(10);
    while(!SPIMasterGetByte(&response));
  }
  return TimestampInPast(timeout);
}
