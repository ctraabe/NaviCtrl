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
#include "mag_calibration.h"
#include "navigation.h"
#include "sd_card.h"
#include "spi_slave.h"
#include "timing.h"
#include "uart1.h"
#include "uart2.h"
#include "ublox.h"
#include "vision.h"


// =============================================================================
// Private data:

#define MAX_CALLBACKS_POW_OF_2 (2)  // 2^2 = 4
#define MAX_CALLBACKS (1 << MAX_CALLBACKS_POW_OF_2)

static volatile Callback callback_buffer_[4] = { 0 };
static volatile size_t callback_buffer_head_ = 0;
static size_t callback_buffer_tail_ = 0;
static uint32_t overrun_counter_ = 0;
#ifndef LOGGING_BUTTON
  static uint32_t mag_calibration_ = 0;
#endif


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

#ifdef LOGGING_BUTTON
  if (button && (button_pv == 0x7FFF))
  {
    if (LoggingActive())
      CloseLogFile();
    else
      OpenLogFile(0);
  }
#else
  // Start and stop magnetometer calibration.
  if (button && (button_pv == 0x7FFF)) mag_calibration_ = !mag_calibration_;
#endif

  // Reset GPS home position.
  // if (button && (button_pv == 0x7FFF)) SetGeodeticHome();

  button_pv = (button_pv << 1) | button;
}

//------------------------------------------------------------------------------
// This function responds to the interrupt triggered when an external device
// pulls down the interrupt line (pin 5 on the FlightCtrl header). This is a
// low-priority interrupt, so some computation can be safely added here.
void FlightCtrlInterruptHandler(void)
{
  WIU_ClearITPendingBit(WIU_Line16);
  VIC_SWITCmd(EXTIT2_ITLine, DISABLE);

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
  UART1Init();
  UART2Init();
  I2CInit();
  SPISlaveInit();

  Wait(100);
  UART1Printf("University of Tokyo NaviCtrl firmware V2");

  ReadEEPROM();
  UBloxInit();
  LSM303DLInit();
  FlightCtrlCommsInit();
  SDCardInit();
  NavigationInit();

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
    // Check for new data from the magnetometer.
    ProcessIncomingLSM303DL();

#ifndef LOGGING_BUTTON
    // Skip the rest of the main loop if mag calibration is ongoing.
    if (MagCalibration(mag_calibration_)) continue;
#endif

    // Check for new data on the GPS UART port.
    ProcessIncomingUBlox();

    // Check for new data from the FlightCtrl.
    if (NewDataFromFlightCtrl())
    {
      ClearNewDataFromFlightCtrlFlag();

      KalmanAccelerometerUpdate();

      UpdateNavigation();

      PrepareFlightCtrlDataExchange();

      RequestLSM303DL();

      // Check if new data has come while processing the data. This indicates
      // that processing did not complete fast enough.
      if (NewDataFromFlightCtrl())
      {
        overrun_counter_++;
      }
    }

    // Check for incoming data on the "update & debug" UART port.
    ProcessIncomingUART1();

    // Check for incoming vision data on the "FligthCtrl" UART port.
    ProcessIncomingUART2();

    // ProcessLogging();

    // Check sensor data freshness.
    CheckUBXFreshness();
    CheckVisionFreshness();
    CheckLSM303DLFreshness();

    // Normally the magnetometer is read every time new data comes from the
    // FlightCtrl. The following statement is a backup that ensures the
    // magnetometer is updated even if there is no connection to the FlightCtrl
    // and also deals with read errors.
    if (LSM303DLDataStale())
    {
      if (MillisSinceTimestamp(LSM303DLLastRequestTimestamp()) > 20)
        RequestLSM303DL();
      if (LSM303DLErrorBits() & LSM303DL_ERROR_BIT_I2C_BUSY)
        I2CReset();
    }

    if (TimestampInPast(led_timer))
    {
      GreenLEDToggle();

      while (TimestampInPast(led_timer)) led_timer += 100;

      // Debug output for GPS and magnetomter. Remove after testing is completed

      // UART1Printf("%+5.2f,%+5.2f,%+5.2f",
      //   MagneticVector()[0],
      //   MagneticVector()[1],
      //   MagneticVector()[2]);

      // UART1Printf("%i,%i,%i",
      //   MagnetometerVector()[0],
      //   MagnetometerVector()[1],
      //   MagnetometerVector()[2]);

      // UART1Printf("%i,%i,%i",
      //   MagnetometerBiasVector()[0],
      //   MagnetometerBiasVector()[1],
      //   MagnetometerBiasVector()[2]);

      // UART1Printf("%f", CurrentHeading());

      // UART1Printf("%f,%f,%f",
      //   (float)(UBXPosLLH()->longitude * 1e-7),
      //   (float)(UBXPosLLH()->latitude * 1e-7),
      //   (float)(UBXPosLLH()->height_above_ellipsoid * 1e-3));

      // UART1PrintfSafe("C:(%+6.2f,%+6.2f,%+6.2f) H:%+4.0f",
      //   VisionPositionVector()[0],
      //   VisionPositionVector()[1],
      //   VisionPositionVector()[2],
      //   CurrentHeading() * 180.0 / 3.141596);

      // UART1PrintfSafe("C:(%+6.2f,%+6.2f,%+6.2f) D:(%+6.2f,%+6.2f,%+6.2f) H:%+4.0f",
      //   PositionVector()[0],
      //   PositionVector()[1],
      //   PositionVector()[2],
      //   NavDeltaPosition(0),
      //   NavDeltaPosition(1),
      //   NavDeltaPosition(2),
      //   CurrentHeading() * 180.0 / 3.141596);

      UART1Printf("%+i,%+i,%+i",
        UBXGeodeticPositionVector()[0],
        UBXGeodeticPositionVector()[1],
        UBXGeodeticPositionVector()[2]);

      // UART1PrintfSafe("%+5.2f,%+5.2f,%+5.2f",
      //   VisionObstacleLocationVector()[0],
      //   VisionObstacleLocationVector()[1],
      //   VisionObstacleLocationVector()[2]);
    }
  }
}
