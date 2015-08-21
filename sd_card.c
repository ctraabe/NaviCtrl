#include "sd_card.h"

#include "91x_lib.h"
#include "diskio.h"  // from libfatfs
#include "timing.h"


// =============================================================================
// Private data:

static DSTATUS status_ = STA_NODISK | STA_NOINIT;


// =============================================================================
// Private function declarations:

static uint32_t SDCardPresent(void);


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
DSTATUS disk_initialize (BYTE pdrv)
{
  if (pdrv != 0) return STA_NODISK | STA_NOINIT;

  Wait(10);
  SDCardCSHigh();

  {
    rx
  }



  return STA_NOINIT;
}

// -----------------------------------------------------------------------------
// This function reads sector(s) from the SD card.
DRESULT disk_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
  return RES_PARERR;
}

// -----------------------------------------------------------------------------
// This function writes sector(s) to the SD card.
DRESULT disk_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
  return RES_PARERR;
}
// -----------------------------------------------------------------------------
// This function controls miscellaneous functions other than generic read/write.
DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void *buff)
{
  return RES_PARERR;
}

DWORD get_fattime (void)
{
  return 0;
}


// =============================================================================
// Private functions:

void SDCardCSHigh(void)
{
  GPIO_WriteBit(GPIO5, GPIO_Pin_4 , Bit_RESET);
}

// -----------------------------------------------------------------------------
void SDCardCSDisable(void) 
{
  GPIO_WriteBit(GPIO5, GPIO_Pin_4 , Bit_SET);
}

// -----------------------------------------------------------------------------
static uint32_t SDCardPresent(void)
{
  return !(GPIO_ReadBit(GPIO5, GPIO_Pin_3));
}

// -----------------------------------------------------------------------------
void UART1_IRQHandler(void)
{
  UART_ClearITPendingBit(UART1, UART_IT_Receive);
  ReceiveUARTData();
}
