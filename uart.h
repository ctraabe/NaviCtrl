#ifndef UART_H_
#define UART_H_


#include <inttypes.h>


#define UARTPrintf(format, ...) UARTPrintf_P(format, ##__VA_ARGS__)


// =============================================================================
// Public functions:

void UARTInit(void);

// -----------------------------------------------------------------------------
// This function mimics printf, but puts the result on the UART stream. It also
// adds the end-of-line characters and checks that the character buffer is not
// exceeded. Note that this function is slow and blocking.
void UARTPrintf_P(const char *format, ...);


#endif  // UART_H_
