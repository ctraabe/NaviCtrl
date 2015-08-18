#include "i2c.h"

#include "91x_lib.h"
#include "config.h"


// =============================================================================
// Private data:

#define F_SCL (100000)

enum I2CMode {
  I2C_MODE_IDLE,
  I2C_MODE_TX,
  I2C_MODE_RX,
  I2C_MODE_TX_THEN_RX
};

static volatile enum I2CMode i2c_mode_ = I2C_MODE_IDLE;
static volatile enum I2CError i2c_error_ = I2C_ERROR_NONE;
static volatile uint8_t rx_destination_len_ = 0, tx_source_len_ = 0;
static volatile uint8_t * rx_destination_ptr_ = 0;
static volatile const uint8_t * tx_source_ptr_ = 0;
static volatile uint8_t register_address_specified_ = 0;

static uint8_t register_address_ = 0x00, slave_address_ = 0x00;
static I2CCallback callback_ptr_ = 0;


// =============================================================================
// Private function declarations:

static void I2CStart(enum I2CMode i2c_mode);
static void I2CStop(void);


// =============================================================================
// Accessors

enum I2CError I2CError(void)
{
  return i2c_error_;
}


// =============================================================================
// Public functions:

// This initialization sets the I2C pin states and clock. It should be performed
// prior to enabling interrupts.
void I2CInit(void)
{
  SCU_APBPeriphClockConfig(__GPIO2, ENABLE);
  SCU_APBPeriphClockConfig(__I2C1,ENABLE);

  GPIO_InitTypeDef gpio_init;

  // Configure pins GPIO2.2 and GPIO2.3 to SCL and SDA
  gpio_init.GPIO_Direction = GPIO_PinOutput;
  gpio_init.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  gpio_init.GPIO_Type = GPIO_Type_OpenCollector;
  gpio_init.GPIO_IPInputConnected = GPIO_IPInputConnected_Enable;
  gpio_init.GPIO_Alternate=GPIO_OutputAlt2;
  GPIO_Init(GPIO2, &gpio_init);

  I2C_InitTypeDef i2c_init;

  i2c_init.I2C_GeneralCall = I2C_GeneralCall_Disable;
  i2c_init.I2C_Ack = I2C_Ack_Enable;
  i2c_init.I2C_CLKSpeed = F_SCL;
  i2c_init.I2C_OwnAddress = 0x00;
  I2C_DeInit(I2C1);
  I2C_Init(I2C1, &i2c_init);
  I2C_Cmd(I2C1, ENABLE);

  I2C_ITConfig(I2C1, ENABLE);
  VIC_Config(I2C1_ITLine, VIC_IRQ, PRIORITY_I2C1);
  VIC_ITCmd(I2C1_ITLine, ENABLE);

  I2C_GenerateSTOP(I2C1, ENABLE);
}

// -----------------------------------------------------------------------------
uint8_t I2CIsIdle(void)
{
  return i2c_mode_ == I2C_MODE_IDLE;
}

// -----------------------------------------------------------------------------
void I2CReset(void)
{
  I2CStop();
}

// -----------------------------------------------------------------------------
enum I2CError I2CRx(uint8_t slave_address, volatile uint8_t *rx_destination_ptr,
  uint8_t rx_destination_len)
{
  return I2CRxThenCallback(slave_address, rx_destination_ptr,
    rx_destination_len, 0);
}

// -----------------------------------------------------------------------------
enum I2CError I2CRxFromRegister(uint8_t slave_address, uint8_t register_address,
  volatile uint8_t *rx_destination_ptr, uint8_t rx_destination_len)
{
  register_address_ = register_address;
  register_address_specified_ = 1;
  return I2CTxThenRx(slave_address, 0, 0, rx_destination_ptr,
    rx_destination_len);
}

// -----------------------------------------------------------------------------
enum I2CError I2CRxThenCallback(uint8_t slave_address,
  volatile uint8_t *rx_destination_ptr, uint8_t rx_destination_len,
  I2CCallback callback_ptr)
{
  if (i2c_mode_ != I2C_MODE_IDLE) return I2C_ERROR_BUSY;
  slave_address_ = slave_address;
  rx_destination_ptr_ = rx_destination_ptr;
  rx_destination_len_ = rx_destination_len;
  callback_ptr_ = callback_ptr;
  i2c_error_ = I2C_ERROR_NONE;
  I2CStart(I2C_MODE_RX);
  return I2C_ERROR_NONE;
}

// -----------------------------------------------------------------------------
enum I2CError I2CTx(uint8_t slave_address, const uint8_t *tx_source_ptr,
  uint8_t tx_source_len)
{
  return I2CTxThenRxThenCallback(slave_address, tx_source_ptr, tx_source_len,
    0, 0, (I2CCallback)0);
}

// -----------------------------------------------------------------------------
enum I2CError I2CTxThenRx(uint8_t slave_address, const uint8_t *tx_source_ptr,
  uint8_t tx_source_len, volatile uint8_t *rx_destination_ptr,
  uint8_t rx_destination_len)
{
  return I2CTxThenRxThenCallback(slave_address, tx_source_ptr, tx_source_len,
    rx_destination_ptr, rx_destination_len, (I2CCallback)0);
}

// -----------------------------------------------------------------------------
enum I2CError I2CTxToRegister(uint8_t slave_address, uint8_t register_address,
  const uint8_t *tx_source_ptr, uint8_t tx_source_len)
{
  register_address_ = register_address;
  register_address_specified_ = 1;
  return I2CTx(slave_address, tx_source_ptr, tx_source_len);
}

// -----------------------------------------------------------------------------
enum I2CError I2CTxThenRxThenCallback(uint8_t slave_address,
  const uint8_t *tx_source_ptr, uint8_t tx_source_len,
  volatile uint8_t *rx_destination_ptr, uint8_t rx_destination_len,
  I2CCallback callback_ptr)
{
  if (i2c_mode_ != I2C_MODE_IDLE) return I2C_ERROR_BUSY;
  slave_address_ = slave_address;
  tx_source_ptr_ = tx_source_ptr;
  tx_source_len_ = tx_source_len;
  rx_destination_ptr_ = rx_destination_ptr;
  rx_destination_len_ = rx_destination_len;
  callback_ptr_ = callback_ptr;
  i2c_error_ = I2C_ERROR_NONE;
  if (rx_destination_ptr_) I2CStart(I2C_MODE_TX_THEN_RX);
  else I2CStart(I2C_MODE_TX);
  return I2C_ERROR_NONE;
}

// -----------------------------------------------------------------------------
// TODO: make a version that takes a limit
void I2CWaitUntilCompletion(void)
{
  while (i2c_mode_ != I2C_MODE_IDLE) continue;
}


// =============================================================================
// Private functions:

static void I2CReadByte(void)
{
  *rx_destination_ptr_ = I2C_ReceiveData(I2C1);
  rx_destination_ptr_++;
  rx_destination_len_--;
}

// -----------------------------------------------------------------------------
// Give a start or repeated start signal.
static void I2CStart(enum I2CMode i2c_mode)
{
  i2c_mode_ = i2c_mode;
  if (rx_destination_len_ > 1) I2C_AcknowledgeConfig(I2C1, ENABLE);
  I2C_GenerateStart(I2C1, ENABLE);
}

// -----------------------------------------------------------------------------
// Give the stop signal.
static void I2CStop(void)
{
  I2C_GenerateSTOP(I2C1, ENABLE);
  i2c_mode_ = I2C_MODE_IDLE;
}

// -----------------------------------------------------------------------------
// Initiate or continue transmission from a buffer.
static void I2CTxBuffer(void)
{
  I2C_SendData(I2C1, *tx_source_ptr_);
  tx_source_ptr_++;
  tx_source_len_--;
}

// -----------------------------------------------------------------------------
// Initiate transmission of a single byte.
static void I2CTxByte(uint8_t byte)
{
  I2C_SendData(I2C1, byte);
}

// -----------------------------------------------------------------------------
// Go to the next communication phase.
static void Next(void)
{
  if (i2c_mode_ == I2C_MODE_TX_THEN_RX && i2c_error_ == I2C_ERROR_NONE)
  {
    I2CStart(I2C_MODE_RX);
  }
  else
  {
    I2CStop();
    if (callback_ptr_) (*callback_ptr_)();
  }
}

// -----------------------------------------------------------------------------
// I2C interrupt indicating that the I2C is active and waiting for the next
// instruction.
void I2C1_IRQHandler(void)
{
  switch (I2C_GetLastEvent(I2C1))
  {
    case I2C_EVENT_MASTER_MODE_SELECT:  // EV5
      if (i2c_mode_ == I2C_MODE_RX)
        I2C_Send7bitAddress(I2C1, slave_address_, I2C_MODE_RECEIVER);
      else
        I2C_Send7bitAddress(I2C1, slave_address_, I2C_MODE_TRANSMITTER);
      break;

    case I2C_EVENT_MASTER_MODE_SELECTED:  // EV6
      // Clear EV6 by by writing to I2C_CR register (for example PE (0x20))
      I2C1->CR |=  0x20;
      if (i2c_mode_ == I2C_MODE_RX) break;
      if (register_address_specified_) {
        I2CTxByte(register_address_);
        register_address_specified_ = 0;
        break;
      }
      // continue

    case I2C_EVENT_MASTER_BYTE_TRANSMITTED:  // EV8
      if (tx_source_len_ > 0) I2CTxBuffer();
      else  Next();
      break;

    case I2C_EVENT_MASTER_BYTE_RECEIVED:  // EV7
      if (rx_destination_len_ == 1) I2C_AcknowledgeConfig(I2C1, DISABLE);
      else if (rx_destination_len_ == 0) Next();
      I2CReadByte();
      break;

    default:
      // Unexpected status message. Send stop.
      i2c_error_ = I2C_ERROR_OTHER;
      Next();
      break;
  }
}
