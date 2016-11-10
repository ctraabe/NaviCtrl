#include "lsm303dl.h"

#include "91x_lib.h"
#include "eeprom.h"
#include "flight_ctrl_comms.h"
#include "i2c.h"
#include "main.h"
#include "timing.h"
#ifdef LOG_DEBUG_TO_SD
  #include "logging.h"
#endif

// =============================================================================
// Private data:

#define LSM303DL_ADDRESS_A (0x30)
#define LSM303DL_ADDRESS_M (0x3C)

#define LSM303DL_RA_CR_M (0x00)
#define LSM303DL_RA_MR_M (0x02)
#define LSM303DL_RA_OUT_M (0x03)
#define LSM303DL_RA_SR_M (0x09)
#define LSM303DL_RA_IR_M (0x0A)
#define LSM303DLM_RA_WHO_AM_I_M (0x0F)  // Only LSM303DLM will respond
#define LSM303DL_MULTI_BYTE_READ (0x80)

#define LSM303DL_CR0_M_DO_75_HZ (0x18)
#define LSM303DL_CR0_M_MS_NORMAL (0x00)
#define LSM303DL_CR1_M_GN_4_0 (0x80)
#define LSM303DL_MR_M_MD_CONTINUOUS (0x00)
#define LSM303DLM_WHO_AM_I_M (0x3C)  // Only LSM303DLM will respond

static enum LSM303DLModel {
  LSM303DL_NOT_PRESENT = 0,
  LSM303DL_MODEL_H,
  LSM303DL_MODEL_M,
} lsm303dl_model_ = LSM303DL_NOT_PRESENT;

static volatile uint32_t unprocessed_data_waiting_ = 0;
static volatile uint8_t magnetometer_raw_[6] = { 0 };
static float magnetic_vector_[3] = { 0.0 };
static int16_t magnetometer_[3] = { 0 };
static uint32_t last_request_timestamp_ = 0, last_update_timepstamp_ = 0;
static uint32_t error_bits_ = LSM303DL_ERROR_BIT_NOT_INITIALIZED;


// =============================================================================
// Private function declarations:

static void DataReceivedCallback(void);


// =============================================================================
// Accessors:

uint32_t LSM303DLDataWaiting(void)
{
  return unprocessed_data_waiting_;
}

// -----------------------------------------------------------------------------
uint32_t LSM303DLErrorBits(void)
{
  return error_bits_;
}

// -----------------------------------------------------------------------------
uint32_t LSM303DLLastRequestTimestamp(void)
{
  return last_request_timestamp_;
}

// -----------------------------------------------------------------------------
uint32_t LSM303DLLastUpdateTimestamp(void)
{
  return last_update_timepstamp_;
}

// -----------------------------------------------------------------------------
const float * MagneticVector(void)
{
  return magnetic_vector_;
}

// -----------------------------------------------------------------------------
const int16_t * MagnetometerVector(void)
{
  return magnetometer_;
}


// =============================================================================
// Public functions:

uint32_t LSM303DLInit(void)
{
  // Determine the compass model.
  uint8_t rx_buffer = 0;
  I2CRxFromRegister(LSM303DL_ADDRESS_M, LSM303DLM_RA_WHO_AM_I_M, &rx_buffer, 1);
  I2CWaitUntilCompletion(10);
  if (rx_buffer == LSM303DLM_WHO_AM_I_M) lsm303dl_model_ = LSM303DL_MODEL_M;
  else if (I2CError() == I2C_ERROR_NONE) lsm303dl_model_ = LSM303DL_MODEL_H;

  // Configure the compass to run continuously.
  const uint8_t tx_buffer[3] = { LSM303DL_CR0_M_DO_75_HZ |
    LSM303DL_CR0_M_MS_NORMAL, LSM303DL_CR1_M_GN_4_0,
    LSM303DL_MR_M_MD_CONTINUOUS };
  I2CTxToRegister(LSM303DL_ADDRESS_M, LSM303DL_RA_CR_M, tx_buffer, 3);
  I2CWaitUntilCompletion(10);

  error_bits_ &= ~LSM303DL_ERROR_BIT_NOT_INITIALIZED;

  return error_bits_;
}

// -----------------------------------------------------------------------------
uint32_t ProcessIncomingLSM303DL(void)
{
  if (!unprocessed_data_waiting_) return 0;

  magnetometer_[1] = (int16_t)(((uint16_t)magnetometer_raw_[0] << 8)
    | magnetometer_raw_[1]);
  if (lsm303dl_model_ == LSM303DL_MODEL_H)
  {
    magnetometer_[0] = (int16_t)(((uint16_t)magnetometer_raw_[2] << 8)
      | magnetometer_raw_[3]);
    magnetometer_[2] = -(int16_t)(((uint16_t)magnetometer_raw_[4] << 8)
      | magnetometer_raw_[5]);
  }
  else
  {
    magnetometer_[2] = -(int16_t)(((uint16_t)magnetometer_raw_[2] << 8)
      | magnetometer_raw_[3]);
    magnetometer_[0] = (int16_t)(((uint16_t)magnetometer_raw_[4] << 8)
      | magnetometer_raw_[5]);
  }

  // The following arranges the data to correspond to the standard body axis.
  magnetic_vector_[0] = (float)(magnetometer_[0] - MagnetometerBiasVector()[0])
    * MagnetometerUnitizerVector()[0];
  magnetic_vector_[1] = (float)(magnetometer_[1] - MagnetometerBiasVector()[1])
    * MagnetometerUnitizerVector()[1];
  magnetic_vector_[2] = (float)(magnetometer_[2] - MagnetometerBiasVector()[2])
    * MagnetometerUnitizerVector()[2];

  unprocessed_data_waiting_ = 0;

  UpdateHeadingCorrectionToFlightCtrl();

#ifdef LOG_DEBUG_TO_SD
  SetNewDataCallback(LogMagnetometerData);
#endif

  return 1;
}

// -----------------------------------------------------------------------------
uint32_t RequestLSM303DL(void)
{
  if (unprocessed_data_waiting_) return LSM303DL_ERROR_BIT_DATA_WAITING;

  if (I2CRxFromRegisterThenCallback(LSM303DL_ADDRESS_M, LSM303DL_RA_OUT_M
    | LSM303DL_MULTI_BYTE_READ, magnetometer_raw_, sizeof(magnetometer_raw_),
    DataReceivedCallback) == I2C_ERROR_NONE)
  {
    error_bits_ &= ~LSM303DL_ERROR_BIT_I2C_BUSY;
  }
  else
  {
    error_bits_ |= LSM303DL_ERROR_BIT_I2C_BUSY;
  }

  last_request_timestamp_ = GetTimestamp();

  return error_bits_;
}


// =============================================================================
// Private functions:

static void DataReceivedCallback(void)
{
  unprocessed_data_waiting_ = 1;
  last_update_timepstamp_ = GetTimestamp();
}
