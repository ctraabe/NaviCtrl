#include "eeprom.h"

#include <string.h>

#include "i2c.h"
#include "timing.h"
#include "uart1.h"
#include "union_types.h"


// =============================================================================
// Private function declarations:

void WriteToEEPROM(const void * data_ptr, size_t data_length);


// =============================================================================
// Private data:

#define EEPROM_I2C_ADDRESS (0xA0)
#define EEPROM_SIZE (65536L)  // 64 kB
#define EEPROM_MEMORY_START_ADDRESS (0x1000)
#define EEPROM_VERSION (2)

// Note: this structure is unpadded (memory aligned)
struct {
  uint32_t version;
  float magnetometer_unitizer[3];
  int16_t magnetometer_bias[3];
  uint32_t magnetometer_calibrated;
} eeprom_;


// =============================================================================
// Accessors:

const int16_t * MagnetometerBiasVector(void)
{
  return eeprom_.magnetometer_bias;
}

// -----------------------------------------------------------------------------
uint32_t MagnetometerCalibrated(void)
{
  return eeprom_.magnetometer_calibrated;
}

// -----------------------------------------------------------------------------
const float * MagnetometerUnitizerVector(void)
{
  return eeprom_.magnetometer_unitizer;
}

// -----------------------------------------------------------------------------
void WriteMagnetometerBiasToEEPROM(int16_t bias[3])
{
  eeprom_.magnetometer_bias[0] = bias[0];
  eeprom_.magnetometer_bias[1] = bias[1];
  eeprom_.magnetometer_bias[2] = bias[2];

  WriteToEEPROM(&eeprom_.magnetometer_bias[0],
    sizeof(eeprom_.magnetometer_bias));
}

// -----------------------------------------------------------------------------
void WriteMagnetometerCalibratedToEEPROM(uint32_t calibrated_flag)
{
  eeprom_.magnetometer_calibrated = calibrated_flag;

  WriteToEEPROM(&eeprom_.magnetometer_calibrated,
    sizeof(eeprom_.magnetometer_calibrated));
}

// -----------------------------------------------------------------------------
void WriteMagnetometerUnitizerToEEPROM(float unitizer[3])
{
  eeprom_.magnetometer_unitizer[0] = unitizer[0];
  eeprom_.magnetometer_unitizer[1] = unitizer[1];
  eeprom_.magnetometer_unitizer[2] = unitizer[2];

  WriteToEEPROM(&eeprom_.magnetometer_unitizer[0],
    sizeof(eeprom_.magnetometer_unitizer));
}


// =============================================================================
// Public functions:

void ReadEEPROM(void)
{
  uint8_t tx_buffer[2] = { 0xFF & (EEPROM_MEMORY_START_ADDRESS >> 8),
    0xFF & EEPROM_MEMORY_START_ADDRESS };  // Big endian

  // Try reading a the EEPROM a few times max.
  for (uint32_t i = 5; --i; )
  {
    I2CTxThenRx(EEPROM_I2C_ADDRESS, tx_buffer, 2, (volatile uint8_t *)&eeprom_,
      sizeof(eeprom_));
    I2CWaitUntilCompletion(100);
    if (I2CError() == I2C_ERROR_NONE) break;
    Wait(100);
  }

  Wait(100);

  if (I2CError() != I2C_ERROR_NONE)
  {
    UART1Printf("eeprom: error reading EEPROM over I2C");
    return;
  }

  // If the versions mismatch, clear the EEPROM and reset the version number.
  if (eeprom_.version != EEPROM_VERSION)
  {
    UART1Printf("eeprom: data in EEPROM is incompatible with this FW version");
    memset(&eeprom_, 0, sizeof(eeprom_));
    eeprom_.version = EEPROM_VERSION;
    WriteToEEPROM(&eeprom_, sizeof(eeprom_));
  }
}


// =============================================================================
// Private functions:

void WriteToEEPROM(const void * data_ptr, size_t data_length)
{
  #define EEPROM_PAGE_LENGTH (32)
  if (data_length == 0) return;

  uint8_t tx_buffer[2 + EEPROM_PAGE_LENGTH];
  uint8_t * tx_ptr = &tx_buffer[0];

  // Determine the target EEPROM memory location.
  union U16Bytes eeprom_address = { EEPROM_MEMORY_START_ADDRESS
    + (uint16_t)((uint32_t)data_ptr - (uint32_t)&eeprom_) };

  // Determine the maximum number of bytes that remain in the EEPROM page
  // starting from the target location.
  uint32_t page_length = eeprom_address.u16 % EEPROM_PAGE_LENGTH;
  if (page_length == 0) page_length = EEPROM_PAGE_LENGTH;

  while (data_length)
  {
    // The number of bytes to be sent is the minimum of the number of bytes
    // left in the page and the number of bytes remaining to be sent.
    uint32_t length = page_length < data_length ? page_length : data_length;

    // Swap address bytes (EEPROM device is big-endian).
    *(tx_ptr++) = eeprom_address.bytes[1];
    *(tx_ptr++) = eeprom_address.bytes[0];

    // Fill up the transmit buffer.
    uint8_t * byte_ptr = (uint8_t *)data_ptr;
    while (page_length-- && data_length)
    {
      *(tx_ptr++) = *(byte_ptr++);
      data_length--;
    }

    I2CTx(EEPROM_I2C_ADDRESS, tx_buffer, 2 + length);
    I2CWaitUntilCompletion(100);
    Wait(100);

    // Prepare pointers and counters for the next page.
    eeprom_address.u16 += length;
    page_length = EEPROM_PAGE_LENGTH;
    tx_ptr = &tx_buffer[0];
  }
}