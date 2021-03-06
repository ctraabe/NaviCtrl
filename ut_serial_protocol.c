#include "ut_serial_protocol.h"

#include <string.h>

#include "crc16.h"
#include "uart1.h"
#include "uart2.h"
#include "union_types.h"
#include "ut_serial_rx.h"


// =============================================================================
// Public functions:

// This function collects an incoming byte that is assumed to be part of a
// message encoded in the UTokyo protocol. This function attempts to place the
// incoming byte into the shared data buffer, but abandons the reception if the
// data buffer is not large enough. The return value indicates whether or not
// more bytes are expected. If so, subsequent bytes should also be passed to
// this function.
enum UARTRxMode UTSerialRx(uint8_t byte, uint8_t * data_buffer)
{
  static uint8_t * rx_ptr = 0;
  static uint8_t bytes_processed = 0, length = 0;
  static uint8_t component_id = 0, message_id = 0;
  static union U16Bytes crc;

  if (bytes_processed == 0)  // First byte is payload length
  {
    if ((UT_HEADER_LENGTH + byte) > UART_DATA_BUFFER_LENGTH) goto RESET;
    length = byte;
    crc.u16 = CRCUpdateCCITT(0xFFFF, byte);
    rx_ptr = data_buffer;
  }
  else if (bytes_processed == 1)  // Second byte is the message ID
  {
    message_id = byte;
    crc.u16 = CRCUpdateCCITT(crc.u16, byte);
  }
  else if (bytes_processed == 2)  // Third byte is the component ID
  {
    component_id = byte;
    crc.u16 = CRCUpdateCCITT(crc.u16, byte);
  }
  else if (bytes_processed < (UT_HEADER_LENGTH - 1 + length))  // Payload
  {
    crc.u16 = CRCUpdateCCITT(crc.u16, byte);
    *rx_ptr++ = byte;
  }
  else if (bytes_processed == (UT_HEADER_LENGTH - 1 + length))  // CRC[0]
  {
    if (byte != crc.bytes[0]) goto RESET;
  }
  else  // CRC[1]
  {
    if (byte == crc.bytes[1]) HandleUTRx(component_id, message_id, data_buffer);
    goto RESET;
  }
  bytes_processed++;
  return UART_RX_MODE_UT_ONGOING;

  RESET:
  bytes_processed = 0;
  return UART_RX_MODE_IDLE;
}

// -----------------------------------------------------------------------------
// This function encodes data into a message using the UTokyo protocol. The
// message must contain at least a destination address and a label. If no
// additional data is necessary, then the source pointer and length can both be
// set to zero.
void UTSerialTx(uint8_t component_id, uint8_t message_id,
  const uint8_t * source, size_t length, enum UARTPort uart_port)
{
  if ((length + 1 + UT_HEADER_LENGTH + 2) > UART_TX_BUFFER_LENGTH) return;

  uint8_t * tx_buffer = 0;
  switch (uart_port)
  {
    case UART_PORT_UART1:
      tx_buffer = RequestUART1TxBuffer();
      break;
    case UART_PORT_UART2:
      tx_buffer = RequestUART2TxBuffer();
      break;
    default:
      break;
  }
  if (!tx_buffer) return;
  uint8_t * tx_ptr = tx_buffer;

  // Copy the start character to the TX buffer;
  *tx_ptr++ = UT_START_CHARACTER;

  // Copy the payload length to the TX buffer.
  *tx_ptr++ = length;

  // Copy the message ID to the TX buffer.
  *tx_ptr++ = message_id;

  // Copy the component ID to the TX buffer.
  *tx_ptr++ = component_id;

  // Copy the payload to the TX buffer.
  memcpy(tx_ptr, source, length);
  tx_ptr += length;

  // Compute the CRC (starting from payload length) and copy to the TX buffer.
  union U16Bytes crc = { 0xFFFF };
  for(size_t i = 1; i < length + UT_HEADER_LENGTH; ++i)
    crc.u16 = CRCUpdateCCITT(crc.u16, tx_buffer[i]);
  *tx_ptr++ = crc.bytes[0];
  *tx_ptr = crc.bytes[1];

  switch (uart_port)
  {
    case UART_PORT_UART1:
      UART1TxBuffer(length + UT_HEADER_LENGTH + 2);
      break;
    case UART_PORT_UART2:
      UART2TxBuffer(length + UT_HEADER_LENGTH + 2);
      break;
    default:
      break;
  }
}
