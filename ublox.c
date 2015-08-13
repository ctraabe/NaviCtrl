#include "ublox.h"

#include <string.h>

#include "91x_lib.h"
#include "config.h"


// =============================================================================
// Private data:

#define UBLOX_BAUD (57600)
#define UBLOX_RX_BUFFER_LENGTH (1 << 8)  // 2^8 = 256
#define UBLOX_DATA_BUFFER_LENGTH (70)
#define UBX_SYNC_CHAR_1 (0xb5)
#define UBX_SYNC_CHAR_2 (0x62)
#define UBX_CLASS_NAV (0x01)
#define UBX_ID_POS_LLH (0x02)
#define UBX_ID_VEL_NED (0x12)
#define UBX_ID_SOL (0x06)

static struct UBXPosLLH
{
  uint32_t gps_ms_time_of_week;
  int32_t longitutde;
  int32_t latitude;
  int32_t height_above_ellipsoid;
  int32_t height_mean_sea_level;
  uint32_t horizontal_accuracy;
  uint32_t vertical_accuracy;
} __attribute__((packed)) ubx_pos_llh_;

static struct UBXVelNED
{
  uint32_t gps_ms_time_of_week;
  int32_t velocity_north;
  int32_t velocity_east;
  int32_t velocity_down;
  uint32_t total_speed;
  uint32_t horizontal_speed;
  int32_t course;
  uint32_t speed_accuracy;
  uint32_t course_accuracy;
} __attribute__((packed)) ubx_vel_ned_;

static struct UBXSol
{
  uint32_t gps_ms_time_of_week;
  int32_t fractional_time_of_week;
  int16_t gps_week;
  uint8_t gps_fix_type;
  uint8_t gps_fix_status_flags;
  int32_t ecef_x_coordinate;
  int32_t ecef_y_coordinate;
  int32_t ecef_z_coordinate;
  uint32_t coordinate_accuracy;
  int32_t ecef_x_velocity;
  int32_t ecef_y_velocity;
  int32_t ecef_z_velocity;
  uint32_t velocity_accuracy;
  uint16_t position_dop;
  uint8_t reserved1;
  uint8_t number_of_satelites_used;
  uint32_t reserved2;
} __attribute__((packed)) ubx_sol_;

static volatile uint8_t rx_buffer_[UBLOX_RX_BUFFER_LENGTH];
static volatile size_t rx_buffer_head_ = 0;
static uint8_t data_buffer_[UBLOX_DATA_BUFFER_LENGTH];


// =============================================================================
// Private function declarations:

static void ProcessUBloxData(uint8_t byte);
static void ReceiveUBloxData(void);


// =============================================================================
// Public functions:

void UBloxInit(void)
{
  SCU_APBPeriphClockConfig(__GPIO6, ENABLE);  // Enable the GPIO6 Clock
  SCU_APBPeriphClockConfig(__UART0, ENABLE);  // Enable the UART0 Clock

  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure pin GPIO6.6 to be UART0 Rx
  GPIO_InitStructure.GPIO_Direction = GPIO_PinInput;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Type = GPIO_Type_PushPull;
  GPIO_InitStructure.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
  GPIO_InitStructure.GPIO_Alternate = GPIO_InputAlt1;  // UART0 Rx
  GPIO_Init(GPIO3, &GPIO_InitStructure);

  // Configure pin GPIO6.6 to be UART0 Tx
  GPIO_InitStructure.GPIO_Direction = GPIO_PinOutput;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Type = GPIO_Type_PushPull;
  GPIO_InitStructure.GPIO_Alternate = GPIO_OutputAlt3;  // UART0 Tx
  GPIO_Init(GPIO3, &GPIO_InitStructure);

  UART_InitTypeDef UART_InitStructure;

  UART_InitStructure.UART_WordLength = UART_WordLength_8D;
  UART_InitStructure.UART_StopBits = UART_StopBits_1;
  UART_InitStructure.UART_Parity = UART_Parity_No ;
  UART_InitStructure.UART_BaudRate = UBLOX_BAUD;
  UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
  UART_InitStructure.UART_Mode = UART_Mode_Tx_Rx;
  UART_InitStructure.UART_FIFO = UART_FIFO_Enable;
  UART_InitStructure.UART_TxFIFOLevel = UART_FIFOLevel_1_2;
  UART_InitStructure.UART_RxFIFOLevel = UART_FIFOLevel_1_2;
  UART_DeInit(UART0);
  UART_Init(UART0, &UART_InitStructure);

  // Enable UART Rx interrupt.
  UART_ITConfig(UART0, UART_IT_Receive , ENABLE);
  VIC_Config(UART0_ITLine, VIC_IRQ, PRIORITY_UART0);

  UART_Cmd(UART0, ENABLE);

  // Enable UART0 interrupt for data reception.
  VIC_ITCmd(UART0_ITLine, ENABLE);

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
  {  // Request NAV-SOL message to be output every measurement cycle.
    const uint8_t tx_buffer[11] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01,
      0x06, 0x01, 0x12, 0x4f };
    UBloxTxBuffer(tx_buffer, 11);
  }
  {  // Request NAV-VELNED message to be output every measurement cycle.
    const uint8_t tx_buffer[11] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01,
      0x12, 0x01, 0x1e, 0x67 };
    UBloxTxBuffer(tx_buffer, 11);
  }
}

// -----------------------------------------------------------------------------
// This function processes bytes that have been read into the Rx ring buffer
// (rx_buffer_) by the Rx interrupt handler. Each byte is passed to the
// appropriate Rx handler, which may place it into the temporary data buffer
// (data_buffer_).
void ProcessIncomingUBlox(void)
{
  static size_t rx_buffer_tail = 0;

  ReceiveUBloxData();

  while (rx_buffer_tail != rx_buffer_head_)
  {
    // Move the ring buffer tail forward.
    rx_buffer_tail = (rx_buffer_tail + 1) % UBLOX_RX_BUFFER_LENGTH;
    ProcessUBloxData(rx_buffer_[rx_buffer_tail]);
  }
}

// -----------------------------------------------------------------------------
// This function sends the contents of buffer to the u-blox device. This
// function blocks program execution until the entire buffer is sent.
void UBloxTxBuffer(const uint8_t * buffer, size_t length)
{
  while (length--)
  {
    while(UART_GetFlagStatus(UART0, UART_FLAG_TxFIFOFull)) continue;
    UART_SendData(UART0, *buffer++);
  }
}


// =============================================================================
// Private functions:

static void DecodeUBloxRx(void)
{

  switch (data_buffer_[3])  // ID
  {
    case UBX_ID_POS_LLH:
      memcpy((uint8_t *)&ubx_pos_llh_, &data_buffer_[5],
        sizeof(struct UBXPosLLH));
      break;
    case UBX_ID_VEL_NED:
      memcpy((uint8_t *)&ubx_vel_ned_, &data_buffer_[5],
        sizeof(struct UBXVelNED));
      break;
    case UBX_ID_SOL:
      memcpy((uint8_t *)&ubx_sol_, &data_buffer_[5], sizeof(struct UBXSol));
      break;
    default:
      break;
  }
}

// -----------------------------------------------------------------------------
static void RecordAndAdvance(uint8_t byte, uint8_t * data_buffer_ptr,
  uint8_t * checksum_a, uint8_t * checksum_b, size_t * bytes_processed)
{
  *data_buffer_ptr = byte;
  *checksum_a += byte;
  *checksum_b += *checksum_a;
  ++*bytes_processed;
}

// -----------------------------------------------------------------------------
static void ProcessUBloxData(uint8_t byte)
{
  static size_t length = 0, bytes_processed = 0;
  static uint8_t checksum_a = 0, checksum_b = 0;
  static uint8_t * data_buffer_ptr = NULL;

  switch (bytes_processed)
  {
    case 0:  // Sync char 1
      if (byte == UBX_SYNC_CHAR_1) ++bytes_processed;
      break;
    case 1:  // Sync char 2
      if (byte == UBX_SYNC_CHAR_2) ++bytes_processed;
      else bytes_processed = 0;
      break;
    case 2:  // Class (NAV)
      checksum_a = 0;
      checksum_b = 0;
      data_buffer_ptr = data_buffer_;
      if (byte == UBX_CLASS_NAV)
        RecordAndAdvance(byte, data_buffer_ptr++, &checksum_a, &checksum_b,
          &bytes_processed);
      else
        bytes_processed = 0;
      break;
    case 4:  // Length (lower byte)
      length = byte > UBLOX_DATA_BUFFER_LENGTH ? 0 : byte + 8;
    case 3:  // Byte should always be 0 (no messages exceed 255 bytes)
      RecordAndAdvance(byte, data_buffer_ptr++, &checksum_a, &checksum_b,
        &bytes_processed);
      break;
    default:
      if (bytes_processed < length - 2)
      {
        RecordAndAdvance(byte, data_buffer_ptr++, &checksum_a, &checksum_b,
          &bytes_processed);
      }
      else if (bytes_processed == length -2)
      {
        if (byte == checksum_a) bytes_processed++;
        else bytes_processed = 0;
      }
      else
      {
        if (byte == checksum_b) DecodeUBloxRx();
        bytes_processed = 0;
      }
      break;
  }
}

// -----------------------------------------------------------------------------
static void ReceiveUBloxData(void)
{
  VIC_ITCmd(UART0_ITLine, DISABLE);
  while (!UART_GetFlagStatus(UART0, UART_FLAG_RxFIFOEmpty))
  {
    rx_buffer_head_ = (rx_buffer_head_ + 1) % UBLOX_RX_BUFFER_LENGTH;
    rx_buffer_[rx_buffer_head_] = UART_ReceiveData(UART0);
  }
  VIC_ITCmd(UART0_ITLine, ENABLE);
}

// -----------------------------------------------------------------------------
void UART0_IRQHandler(void)
{
  UART_ClearITPendingBit(UART0, UART_IT_Receive);
  ReceiveUBloxData();
}
