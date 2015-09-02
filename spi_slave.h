#ifndef SPI_SLAVE_H_
#define SPI_SLAVE_H_


#include <inttypes.h>
#include <stddef.h>


// =============================================================================
// Public functions:

void SPISlaveInit(void);
void SPISlaveHandler(void);
// TODO: Remove
char * SPITemp(void);
uint32_t SPIBytesReceived(void);


#endif  // SPI_SLAVE_H_
