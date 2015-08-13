#include "ublox.h"

#include "91x_lib.h"
#include "config.h"


// =============================================================================
// Private data:

#define UBLOX_BAUD (57600)

struct UBXPosLLH
{
  uint32_t gps_ms_time_of_week;
  int32_t longitutde;
  int32_t latitude;
  int32_t height_above_ellipsoid;
  int32_t height_mean_sea_level;
  uint32_t horizontal_accuracy;
  uint32_t vertical_accuracy;
} __attribute__((packed));

struct UBXVelNED
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
} __attribute__((packed));

struct UBXSol
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
} __attribute__((packed));


// =============================================================================
// Public functions:

void UBLOXInit(void)
{
  SCU_APBPeriphClockConfig(__GPIO6, ENABLE);  // Enable the GPIO6 Clock
  SCU_APBPeriphClockConfig(__UART0, ENABLE);  // Enable the UART1 Clock

  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure pin GPIO6.6 to be UART0 Rx
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Direction = GPIO_PinInput;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Type = GPIO_Type_PushPull;
  GPIO_InitStructure.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
  GPIO_InitStructure.GPIO_Alternate = GPIO_InputAlt1;  // UART0 Rx
  GPIO_Init (GPIO3, &GPIO_InitStructure);

  // Configure pin GPIO6.6 to be UART0 Tx
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Direction = GPIO_PinOutput;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Type = GPIO_Type_PushPull;
  GPIO_InitStructure.GPIO_Alternate = GPIO_OutputAlt3;  // UART0 Tx
  GPIO_Init (GPIO3, &GPIO_InitStructure);

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
}
