#include "main.h"

#include "91x_lib.h"
#include "eeprom.h"
#include "flight_ctrl_comms.h"
#include "i2c.h"
#include "irq_priority.h"
#include "kalman_filter.h"
#include "led.h"
#include "logging.h"
#include "lsm303dl.h"
#include "sd_card.h"
#include "spi_slave.h"
#include "timing.h"
#include "uart.h"
#include "ublox.h"


// =============================================================================
// Private data:

#define MAX_CALLBACKS_POW_OF_2 (2)  // 2^2 = 4
#define MAX_CALLBACKS (1 << MAX_CALLBACKS_POW_OF_2)

static volatile Callback callback_buffer_[4] = { 0 };
static volatile size_t callback_buffer_head_ = 0;
static size_t callback_buffer_tail_ = 0;


// =============================================================================
// Private function declarations:

int main(void) __attribute__ ((noreturn));


// =============================================================================
// Public functions:

void FiftyHzInterruptHandler(void)
{
  VIC_SWITCmd(EXTIT3_ITLine, DISABLE);

  uint16_t button = GPIO_ReadBit(GPIO3, GPIO_Pin_1);
  static uint16_t button_pv = 0;
  if (button && (button_pv == 0x7FFF))
  {
    if (LoggingActive())
    {
      CloseLogFile();
    }
    else
    {
      OpenLogFile(0);
    }
  }
  button_pv = (button_pv << 1) | button;

  ProcessIncomingUART();
}

//------------------------------------------------------------------------------
// This function responds to the interrupt triggered when the FlightCtrl pulls
// down the interrupt line. It signals that new IMU data will be coming soon, so
// the NaviCtrl should prepare by updating the GPS and magnetometer data. This
// is a low-priority interrupt, so some computation can be safely added here.
void FlightCtrlInterruptHandler(void)
{
  WIU_ClearITPendingBit(WIU_Line16);
  VIC_SWITCmd(EXTIT2_ITLine, DISABLE);

  LSM303DLReadMag();
  ProcessIncomingUBlox();

  // Prepare volatile IMU data for the Kalman filter.
  float accelerometer[3] = { Accelerometer(X_BODY_AXIS) * GRAVITY_ACCELERATION,
    Accelerometer(Y_BODY_AXIS) * GRAVITY_ACCELERATION,
    Accelerometer(Z_BODY_AXIS) * GRAVITY_ACCELERATION };
  float gyro[3] = { Gyro(X_BODY_AXIS), Gyro(Y_BODY_AXIS), Gyro(Z_BODY_AXIS) };

  KalmanTimeUpdate(gyro, accelerometer);
  KalmanAccelerometerUpdate(accelerometer);

  PrepareFlightCtrlDataExchange();
}

//------------------------------------------------------------------------------
// This is a low-priority interrupt that is triggered by high-frequency data
// collecting interrupts such as SPI and I2C. It enables the data processing to
// be performed at a lower priority than the data collection, but at a higher
// priority that slower computations.
void NewDataInterruptHandler(void)
{
  VIC_SWITCmd(EXTIT0_ITLine, DISABLE);

  while (callback_buffer_head_ != callback_buffer_tail_)
  {
    callback_buffer_tail_ = (callback_buffer_tail_ + 1) % MAX_CALLBACKS;
    (*callback_buffer_[callback_buffer_tail_])();
  }
}

// -----------------------------------------------------------------------------
// This puts a callback into the callback ring buffer.
void SetNewDataCallback(Callback callback)
{
  if (!callback) return;
  callback_buffer_head_ = (callback_buffer_head_ + 1) % MAX_CALLBACKS;
  callback_buffer_[callback_buffer_head_] = callback;
  VIC_SWITCmd(EXTIT0_ITLine, ENABLE);
}


// =============================================================================
// Private functions:

// Configure the interrupt vector.
static void VICConfig(void)
{
  // Enable the AHB (advanced high-performance but) clock for VIC (vectored
  // interrupt controller).
  SCU_AHBPeriphClockConfig(__VIC,ENABLE);
  // Reset the VIC registers (to their default reset values).
  VIC_DeInit();
  // Initialize VICs default vector registers.
  VIC_InitDefaultVectors();
  // Reset the wakeup interrupt unit registers.
  WIU_DeInit();
}

//------------------------------------------------------------------------------
static void ExternalButtonInit(void)
{
  SCU_APBPeriphClockConfig(__GPIO3 ,ENABLE);

  GPIO_InitTypeDef gpio_init;

  // Configure P3.1 -> external button as an input pin.
  gpio_init.GPIO_Direction = GPIO_PinInput;
  gpio_init.GPIO_Pin = GPIO_Pin_1;
  gpio_init.GPIO_Type = GPIO_Type_PushPull;
  gpio_init.GPIO_IPInputConnected = GPIO_IPInputConnected_Disable;
  gpio_init.GPIO_Alternate = GPIO_InputAlt1;
  GPIO_Init(GPIO3, &gpio_init);
}

//------------------------------------------------------------------------------
int main(void)
{
  VICConfig();
  TimingInit();
  LEDInit();
  UARTInit();
  I2CInit();
  SPISlaveInit();

  UARTPrintf("University of Tokyo NaviCtrl firmware V2");

  ReadEEPROM();
  UBloxInit();
  LSM303DLInit();
  FlightCtrlCommsInit();
  SDCardInit();
  LoggingInit();

  ExternalButtonInit();

  // Enable the "new data" interrupt.
  VIC_Config(EXTIT0_ITLine, VIC_IRQ, IRQ_PRIORITY_NEW_DATA);
  VIC_ITCmd(EXTIT0_ITLine, ENABLE);

  // Enable the 50Hz Interrupt.
  VIC_Config(EXTIT3_ITLine, VIC_IRQ, IRQ_PRIORITY_50HZ);
  VIC_ITCmd(EXTIT3_ITLine, ENABLE);

  // Main loop.
  uint32_t led_timer = GetTimestamp();
  for (;;)
  {
    // TODO: add magnetometer timer for operation independent from FC
/*
    uint32_t counter = (1 << 16), timer = GetTimestamp();
    char test_msg[16] = "0123456789abcdef";
    for ( ; sd_write_test && counter != 0; --counter) LogWrite(test_msg, 16);
    if (counter == 0)
    {
      CloseLogFile();
      RedLEDOff();
      sd_write_test = 0;
      UARTPrintf("SD write test finished in %i ms", MillisSinceTimestamp(timer));
    }
*/
    ProcessLogging();

    if (TimestampInPast(led_timer))
    {
      GreenLEDToggle();
      led_timer += 250;
    }
  }
}
