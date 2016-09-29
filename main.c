#include "main.h"

#include "91x_lib.h"
#include "eeprom.h"
#include "flight_ctrl_comms.h"
#include "i2c.h"
#include "irq_priority.h"
#include "led.h"
#include "logging.h"
#include "lsm303dl.h"
#include "mag_calibration.h"
#include "navigation.h"
#include "sd_card.h"
#include "spi_slave.h"
#include "timing.h"
#include "uart.h"
#include "uart2.h"
#ifndef VISION
  #include "ublox.h"
#else
  #include "kalman_filter.h"
  #include "vision.h"
#endif


// =============================================================================
// Private data:

#define MAX_CALLBACKS_POW_OF_2 (2)  // 2^2 = 4
#define MAX_CALLBACKS (1 << MAX_CALLBACKS_POW_OF_2)

static volatile Callback callback_buffer_[4] = { 0 };
static volatile size_t callback_buffer_head_ = 0;
static size_t callback_buffer_tail_ = 0;
static volatile uint32_t flight_ctrl_interrupt_ = 0;
static uint32_t overrun_counter_ = 0;
static uint32_t mag_calibration_ = 0;


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
  if (button && (button_pv == 0x7FFF)) mag_calibration_ = !mag_calibration_;
  button_pv = (button_pv << 1) | button;
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

// -----------------------------------------------------------------------------
// This puts a callback into the callback ring buffer.
void SetFlightCtrlInterrupt(void)
{
  flight_ctrl_interrupt_ = 1;
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
static uint32_t MagCalibration(uint32_t mag_calibration)
{
  static uint32_t mag_calibration_pv = 0, mag_calibration_timer = 0;

  if (mag_calibration)
  {
    if (!mag_calibration_pv)
    {
      MagCalibrationInit(MagnetometerVector());
      mag_calibration_timer = GetTimestampMillisFromNow(20);
      UARTPrintf("Calibration start");
    }

    // Take a sample every 20 ms.
    while (!TimestampInPast(mag_calibration_timer)) continue;
    mag_calibration_timer += 20;

    LSM303DLReadMag();
    I2CWaitUntilCompletion(10);
    MagCalibrationAddSample(MagnetometerVector());
  }
  else if (mag_calibration_pv)
  {
    int16_t bias[3];
    float unitizer[3];
    MagCalibrationCompute(unitizer, bias);
    WriteMagnetometerUnitizerToEEPROM(unitizer);
    WriteMagnetometerBiasToEEPROM(bias);
    WriteMagnetometerCalibratedToEEPROM(1);
    UARTPrintf("unitizer: %f, %f, %f", unitizer[0], unitizer[1], unitizer[2]);
    UARTPrintf("bias: %i, %i, %i", bias[0], bias[1], bias[2]);
  }

  mag_calibration_pv = mag_calibration;
  return mag_calibration;
}

//------------------------------------------------------------------------------
int main(void)
{
  VICConfig();
  TimingInit();
  LEDInit();
  UARTInit();
  UART2Init();
  I2CInit();
  SPISlaveInit();

  Wait(100);
  UARTPrintf("University of Tokyo NaviCtrl firmware V2");

  ReadEEPROM();
#ifndef VISION
  UBloxInit();
#else
  VisionInit();
#endif
  LSM303DLInit();
  FlightCtrlCommsInit();
  SDCardInit();
  NavigationInit();

  ExternalButtonInit();

  {
    int16_t bias[3] = { -35, -160, 10 };
    float unitizer[3] = { 1.0, 1.0, 1.0 };
    WriteMagnetometerUnitizerToEEPROM(unitizer);
    WriteMagnetometerBiasToEEPROM(bias);
  }

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
    if (MagCalibration(mag_calibration_)) continue;

#ifndef VISION
    ProcessIncomingUBlox();
#else
    if (ProcessIncomingVision())
    {
      KalmanVisionUpdate();
#ifdef LOG_DEBUG_TO_SD
      SetNewDataCallback(LogVisionData);
      SetNewDataCallback(LogKalmanData);
#endif
    }
#endif

    if (flight_ctrl_interrupt_)
    {
      flight_ctrl_interrupt_ = 0;

      LSM303DLReadMag();

#ifdef VISION
      KalmanTimeUpdate();
#endif

      UpdateNavigation();

      PrepareFlightCtrlDataExchange();

      if (flight_ctrl_interrupt_)
      {
        overrun_counter_++;
      }
    }

    ProcessIncomingUART();

#ifdef LOG_DEBUG_TO_SD
    ProcessLogging();
#endif

    if (TimestampInPast(led_timer))
    {
      GreenLEDToggle();
      led_timer += 100;

      UARTPrintf("%0.2f,%0.2f,%0.2f",
        MagneticVector()[0],
        MagneticVector()[1],
        MagneticVector()[2]);

      UARTPrintf("%i,%i,%i",
        MagnetometerVector()[0],
        MagnetometerVector()[1],
        MagnetometerVector()[2]);

      UARTPrintf("%i,%i,%i",
        MagnetometerBiasVector()[0],
        MagnetometerBiasVector()[1],
        MagnetometerBiasVector()[2]);

      UARTPrintf("%f", CurrentHeading());
    }
  }
}
