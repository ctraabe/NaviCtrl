#ifndef I2C_H_
#define I2C_H_


#include <inttypes.h>
#include <stddef.h>


typedef void (*I2CCallback)(void);

enum I2CError {
  I2C_ERROR_NONE = 0,
  I2C_ERROR_ACK,
  I2C_ERROR_NACK,
  I2C_ERROR_NO_REPLY,
  I2C_ERROR_OTHER,
  I2C_ERROR_BUSY,
  I2C_ERROR_BUS,
};


// =============================================================================
// Accessors:

enum I2CError I2CError(void);


// =============================================================================
// Public functions:

// This initialization sets the I2C pin states and clock. It should be performed
// prior to enabling interrupts.
void I2CInit(void);

// -----------------------------------------------------------------------------
uint32_t I2CIsIdle(void);

// -----------------------------------------------------------------------------
void I2CReset(void);

// -----------------------------------------------------------------------------
enum I2CError I2CRx(uint8_t slave_address, volatile uint8_t *rx_destination_ptr,
  size_t rx_destination_len);

// -----------------------------------------------------------------------------
enum I2CError I2CRxFromRegister(uint8_t slave_address, uint8_t register_address,
  volatile uint8_t *rx_destination_ptr, size_t rx_destination_len);

// -----------------------------------------------------------------------------
enum I2CError I2CRxFromRegisterThenCallback(uint8_t slave_address,
  uint8_t register_address, volatile uint8_t *rx_destination_ptr,
  size_t rx_destination_len, I2CCallback callback_ptr);

// -----------------------------------------------------------------------------
enum I2CError I2CRxThenCallback(uint8_t slave_address,
  volatile uint8_t *rx_destination_ptr, size_t rx_destination_len,
  I2CCallback callback_ptr);

// -----------------------------------------------------------------------------
enum I2CError I2CTx(uint8_t slave_address, const uint8_t *tx_source_ptr,
  const size_t tx_source_len);

// -----------------------------------------------------------------------------
enum I2CError I2CTxToRegister(uint8_t slave_address, uint8_t register_address,
  const uint8_t *tx_source_ptr, size_t tx_source_len);

// -----------------------------------------------------------------------------
enum I2CError I2CTxThenRx(uint8_t slave_address, const uint8_t *tx_source_ptr,
  size_t tx_source_len, volatile uint8_t *rx_destination_ptr,
  size_t rx_destination_len);

// -----------------------------------------------------------------------------
enum I2CError I2CTxThenRxThenCallback(uint8_t slave_address,
  const uint8_t *tx_source_ptr, size_t tx_source_len,
  volatile uint8_t *rx_destination_ptr, size_t rx_destination_len,
  I2CCallback callback_ptr);

// -----------------------------------------------------------------------------
uint32_t I2CWaitUntilCompletion(uint32_t time_limit_ms);

// -----------------------------------------------------------------------------
// I2C interrupt indicating that the I2C is active and waiting for the next
// instruction.
void I2CHandler(void);


#endif  // I2C_H_
