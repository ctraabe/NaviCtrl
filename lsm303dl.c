#include "lsm303dl.h"

#include "i2c.h"
#include "timing.h"
// TODO: remove
#include "91x_lib.h"
#include "uart.h"


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

static int16_t magnetometer_[3] = { 0 };
static volatile uint8_t magnetometer_raw_[6] = { 0 };


// =============================================================================
// Accessors:

int16_t * Magnetometer(void)
{
  return magnetometer_;
}


// =============================================================================
// Public functions:

void LSM303DLInit(void)
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
}

// -----------------------------------------------------------------------------
void LSM303DLReadMag(void)
{
  I2CRxFromRegister(LSM303DL_ADDRESS_M, LSM303DL_RA_OUT_M
    | LSM303DL_MULTI_BYTE_READ, magnetometer_raw_, sizeof(magnetometer_raw_));
  I2CWaitUntilCompletion(10);

  magnetometer_[0] = (int16_t)(((uint16_t)magnetometer_raw_[0] << 8)
    | magnetometer_raw_[1]);
  if (lsm303dl_model_ == LSM303DL_MODEL_H)
  {
    magnetometer_[1] = (int16_t)(((uint16_t)magnetometer_raw_[2] << 8)
      | magnetometer_raw_[3]);
    magnetometer_[2] = (int16_t)(((uint16_t)magnetometer_raw_[4] << 8)
      | magnetometer_raw_[5]);
  }
  else
  {
    magnetometer_[2] = (int16_t)(((uint16_t)magnetometer_raw_[2] << 8)
      | magnetometer_raw_[3]);
    magnetometer_[1] = (int16_t)(((uint16_t)magnetometer_raw_[4] << 8)
      | magnetometer_raw_[5]);
  }
}