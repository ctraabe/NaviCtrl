#ifndef CRC16_H_
#define CRC16_H_


#include <inttypes.h>


// This is the same CRC that is used in MAVLink. It uses a polynomial
// represented by 0x1021 with both the input and output reflected. The CRC
// should be initialized to 0xFFFF.
static inline uint16_t CRCUpdateCCITT(uint16_t crc, uint8_t data)
{
  data ^= (uint8_t)(crc & 0xFF);
  data ^= (data << 4);
  return (crc >> 8) ^ (data << 8) ^ (data << 3) ^ (data >> 4);
}


#endif  // CRC16_H_
