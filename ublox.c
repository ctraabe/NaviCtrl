// This file initializes a UBlox unit to send periodic updates over UART at
// 57600 baud. The STR91x UART has an 8-byte input buffer. When the input buffer
// becomes more than 3/4 full, an interrupt occurs. The interrupt handler (in
// this file) moves the data from the UART buffer into RAM (rx_buffer_) for
// later processing. 

// The received data is put into a receive buffer for later processing.

#include "ublox.h"

#include <string.h>

#include "91x_lib.h"
#include "irq_priority.h"
#include "logging.h"
#include "main.h"
#include "timing.h"


// =============================================================================
// Private data:

#define UBLOX_INITIAL_BAUD (9600)
#define UBLOX_OPERATING_BAUD (57600)
#define UBLOX_RX_BUFFER_LENGTH (1 << 8)  // 2^8 = 256
#define UBLOX_DATA_BUFFER_LENGTH (sizeof(struct UBXSol))  // Largest payload

#define UBX_SYNC_CHAR_1 (0xb5)
#define UBX_SYNC_CHAR_2 (0x62)
#define UBX_CLASS_NAV (0x01)
#define UBX_ID_POS_LLH (0x02)
#define UBX_ID_VEL_NED (0x12)
#define UBX_ID_SOL (0x06)
#define UBX_ID_TIME_UTC (0x21)

static volatile uint8_t rx_buffer_[UBLOX_RX_BUFFER_LENGTH];
static volatile size_t rx_buffer_head_ = 0;
static uint8_t data_buffer_[UBLOX_DATA_BUFFER_LENGTH];

static struct UBXPosLLH ubx_pos_llh_;
static struct UBXVelNED ubx_vel_ned_;
static struct UBXSol ubx_sol_;
static struct UBXTimeUTC ubx_time_utc_;


// =============================================================================
// Private function declarations:

static void ProcessIncomingUBloxByte(uint8_t byte);
static void ReceiveUBloxData(void);
static void UART0Init(uint32_t baud_rate);
static void UBloxTxBuffer(const uint8_t * buffer, size_t length);


// =============================================================================
// Accessors:

const struct UBXPosLLH * UBXPosLLH(void)
{
  return &ubx_pos_llh_;
}

// -----------------------------------------------------------------------------
const struct UBXVelNED * UBXVelNED(void)
{
  return &ubx_vel_ned_;
}

// -----------------------------------------------------------------------------
const struct UBXSol * UBXSol(void)
{
  return &ubx_sol_;
}

// -----------------------------------------------------------------------------
const struct UBXTimeUTC * UBXTimeUTC(void)
{
  return &ubx_time_utc_;
}


// =============================================================================
// Public functions:

void UBloxInit(void)
{
  SCU_APBPeriphClockConfig(__GPIO6, ENABLE);  // Enable the GPIO6 Clock
  SCU_APBPeriphClockConfig(__UART0, ENABLE);  // Enable the UART0 Clock

  GPIO_InitTypeDef gpio_init;

  // Configure pin GPIO6.6 to be UART0 Rx
  gpio_init.GPIO_Direction = GPIO_PinInput;
  gpio_init.GPIO_Pin = GPIO_Pin_6;
  gpio_init.GPIO_Type = GPIO_Type_PushPull;
  gpio_init.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
  gpio_init.GPIO_Alternate = GPIO_InputAlt1;  // UART0 Rx
  GPIO_Init(GPIO6, &gpio_init);

  // Configure pin GPIO6.6 to be UART0 Tx
  gpio_init.GPIO_Direction = GPIO_PinOutput;
  gpio_init.GPIO_Pin = GPIO_Pin_7;
  gpio_init.GPIO_Type = GPIO_Type_PushPull;
  gpio_init.GPIO_Alternate = GPIO_OutputAlt3;  // UART0 Tx
  GPIO_Init(GPIO6, &gpio_init);

  UART0Init(UBLOX_INITIAL_BAUD);

  // Enable UART Rx interrupt.
  UART_ITConfig(UART0, UART_IT_Receive, ENABLE);
  VIC_Config(UART0_ITLine, VIC_IRQ, IRQ_PRIORITY_UART0);
  VIC_ITCmd(UART0_ITLine, ENABLE);

  {
    // Set the port to UART UBX @ 57600.
    const uint8_t tx_buffer[28] = { 0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01,
      0x00, 0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0xe1, 0x00, 0x00, 0x01,
      0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd6, 0x8d };
    UBloxTxBuffer(tx_buffer, 28);
  }

  Wait(150);
  UART0Init(UBLOX_OPERATING_BAUD);

  {  // Configure USB for UBX input with no output.
    const uint8_t tx_buffer[28] = { 0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x03,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x8c };
    UBloxTxBuffer(tx_buffer, 28);
  }
  {  // Set antenna flags to 0x0b and pins to 0x380f.
    const uint8_t tx_buffer[12] = { 0xb5, 0x62, 0x06, 0x13, 0x04, 0x00, 0x0b,
      0x00, 0x0f, 0x38, 0x6f, 0x4f };
    UBloxTxBuffer(tx_buffer, 12);
  }
  {  // Set measurement period to 200ms (5Hz) with UTC reference.
    const uint8_t tx_buffer[14] = { 0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xc8,
      0x00, 0x01, 0x00, 0x00, 0x00, 0xdd, 0x68 };
    UBloxTxBuffer(tx_buffer, 14);
  }
  {  // Configure TimPulse.
    const uint8_t tx_buffer[28] = { 0xb5, 0x62, 0x06, 0x07, 0x14, 0x00, 0x40,
      0x42, 0x0f, 0x00, 0x90, 0x86, 0x03, 0x00, 0xff, 0x01, 0x00, 0x00, 0x32,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0x70 };
    UBloxTxBuffer(tx_buffer, 28);
  }
  {  // Configure SBAS.
    const uint8_t tx_buffer[16] = { 0xb5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03,
      0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2b, 0xbd };
    UBloxTxBuffer(tx_buffer, 16);
  }
  {  // Configure navigation engine.
    const uint8_t tx_buffer[44] = { 0xb5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xff,
      0xff, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x08,
      0x3c, 0x50, 0x00, 0x32, 0x00, 0x23, 0x00, 0x23, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97,
      0xfa };
    UBloxTxBuffer(tx_buffer, 44);
  }
  {  // Configure navigation engine expert settings.
    const uint8_t tx_buffer[48] = { 0xb5, 0x62, 0x06, 0x23, 0x28, 0x00, 0x00,
      0x00, 0x4c, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x10, 0x14,
      0x00, 0x01, 0x00, 0x00, 0x00, 0xf8, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0xc9, 0xea };
    UBloxTxBuffer(tx_buffer, 48);
  }
  {  // Request NAV-POSLLH message to be output every measurement cycle.
    const uint8_t tx_buffer[11] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01,
      0x02, 0x01, 0x0e, 0x47 };
    UBloxTxBuffer(tx_buffer, 11);
  }
  {  // Request NAV-VELNED message to be output every measurement cycle.
    const uint8_t tx_buffer[11] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01,
      0x12, 0x01, 0x1e, 0x67 };
    UBloxTxBuffer(tx_buffer, 11);
  }
/*
  {  // Request NAV-SOL message to be output every measurement cycle.
    const uint8_t tx_buffer[11] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01,
      0x06, 0x01, 0x12, 0x4f };
    UBloxTxBuffer(tx_buffer, 11);
  }
*/
  {  // Request Time-UTC message to be output every 5 measurement cycles.
    const uint8_t tx_buffer[11] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01,
      0x21, 0x05, 0x31, 0x89 };
    UBloxTxBuffer(tx_buffer, 11);
  }
}

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_buffer_) by the Rx interrupt handler.
void ProcessIncomingUBlox(void)
{
  static size_t rx_buffer_tail = 0;

  // Disable the UART interrupt, empty the UART Rx buffer, then re-enable.
  VIC_ITCmd(UART0_ITLine, DISABLE);
  ReceiveUBloxData();
  VIC_ITCmd(UART0_ITLine, ENABLE);

  while (rx_buffer_tail != rx_buffer_head_)
  {
    // Move the ring buffer tail forward.
    rx_buffer_tail = (rx_buffer_tail + 1) % UBLOX_RX_BUFFER_LENGTH;
    ProcessIncomingUBloxByte(rx_buffer_[rx_buffer_tail]);
  }
}


// =============================================================================
// Private functions:

static inline void UpdateChecksum(uint8_t byte, uint8_t * checksum_a,
  uint8_t * checksum_b)
{
  *checksum_a += byte;
  *checksum_b += *checksum_a;
}

// -----------------------------------------------------------------------------
static void CopyUBloxMessage(uint8_t id)
{
  switch (id)
  {
    case UBX_ID_POS_LLH:
      memcpy(&ubx_pos_llh_, &data_buffer_[0], sizeof(struct UBXPosLLH));
      SetNewDataCallback(LogUBXPosLLH);
      break;
    case UBX_ID_VEL_NED:
      memcpy(&ubx_vel_ned_, &data_buffer_[0], sizeof(struct UBXVelNED));
      SetNewDataCallback(LogUBXVelNED);
      break;
    case UBX_ID_SOL:
      memcpy(&ubx_sol_, &data_buffer_[0], sizeof(struct UBXSol));
      SetNewDataCallback(LogUBXSol);
      break;
    case UBX_ID_TIME_UTC:
      memcpy(&ubx_time_utc_, &data_buffer_[0], sizeof(struct UBXTimeUTC));
      SetNewDataCallback(LogUBXTimeUTC);
      break;
  }
}

// -----------------------------------------------------------------------------
static void ProcessIncomingUBloxByte(uint8_t byte)
{
  static size_t bytes_processed = 0, payload_length = 0;
  static uint8_t id, checksum_a, checksum_b;
  static uint8_t * data_buffer_ptr = NULL;

  switch (bytes_processed)
  {
    case 0:  // Sync char 1
      if (byte != UBX_SYNC_CHAR_1) goto RESET;
      break;
    case 1:  // Sync char 2
      if (byte != UBX_SYNC_CHAR_2) goto RESET;
      break;
    case 2:  // Class (NAV)
      if (byte != UBX_CLASS_NAV) goto RESET;
      checksum_a = byte;
      checksum_b = byte;
      break;
    case 3:  // ID
      id = byte;
      UpdateChecksum(byte, &checksum_a, &checksum_b);
      break;
    case 4:  // Payload length (lower byte)
      if (byte > UBLOX_DATA_BUFFER_LENGTH) goto RESET;
      data_buffer_ptr = &data_buffer_[0];
    case 5:  // Payload length (upper byte should always be zero)
      if (byte != 0) goto RESET;
      UpdateChecksum(byte, &checksum_a, &checksum_b);
      break;
    default:  // Payload or checksum
      if (bytes_processed < (6 + payload_length))  // Payload
      {
        *data_buffer_ptr++ = byte;
        UpdateChecksum(byte, &checksum_a, &checksum_b);
      }
      else if (bytes_processed == (6 + payload_length))  // Checksum A
      {
        if (byte != checksum_a) goto RESET;
      }
      else  // Checksum B
      {
        if (byte == checksum_b) CopyUBloxMessage(id);
        goto RESET;
      }
      break;
  }
  bytes_processed++;
  return;

  RESET:
  bytes_processed = 0;
}

// -----------------------------------------------------------------------------
static void ReceiveUBloxData(void)
{
  while (!UART_GetFlagStatus(UART0, UART_FLAG_RxFIFOEmpty))
  {
    rx_buffer_head_ = (rx_buffer_head_ + 1) % UBLOX_RX_BUFFER_LENGTH;
    rx_buffer_[rx_buffer_head_] = UART_ReceiveData(UART0);
  }
}

// -----------------------------------------------------------------------------
// This function sends the contents of buffer to the u-blox device. This
// function blocks program execution until the entire buffer is sent.
static void UBloxTxBuffer(const uint8_t * buffer, size_t length)
{
  while (length--)
  {
    while(UART_GetFlagStatus(UART0, UART_FLAG_TxFIFOFull)) continue;
    UART_SendData(UART0, *buffer++);
  }
}

// -----------------------------------------------------------------------------
static void UART0Init(uint32_t baud_rate)
{
  UART_InitTypeDef uart_init;

  uart_init.UART_WordLength = UART_WordLength_8D;
  uart_init.UART_StopBits = UART_StopBits_1;
  uart_init.UART_Parity = UART_Parity_No ;
  uart_init.UART_BaudRate = baud_rate;
  uart_init.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
  uart_init.UART_Mode = UART_Mode_Tx_Rx;
  uart_init.UART_FIFO = UART_FIFO_Enable;
  uart_init.UART_TxFIFOLevel = UART_FIFOLevel_1_4;
  uart_init.UART_RxFIFOLevel = UART_FIFOLevel_3_4;
  UART_DeInit(UART0);
  UART_Init(UART0, &uart_init);
  UART_Cmd(UART0, ENABLE);
}

// -----------------------------------------------------------------------------
void UBloxUARTHandler(void)
{
  UART_ClearITPendingBit(UART0, UART_IT_Receive);
  ReceiveUBloxData();
}
