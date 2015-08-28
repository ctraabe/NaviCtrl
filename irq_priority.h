#ifndef _IRQ_PRIORITY_H
#define _IRQ_PRIORITY_H


// Interrupts in VIC0 are always higher priority than interrupts in VIC1, but
// the 16 interrupts within each VIC can be re-prioritized from 0 (highest) to
// 15 (lowest).


// =============================================================================
// VIC0 Priorities (higher priority than VIC1):
#define IRQ_PRIORITY_TIMER1 0  // VIC0.5: Millisecond timer


// =============================================================================
// VIC1 Priorities (higher priority than VIC1):
#define IRQ_PRIORITY_UART0 2  // VIC1.0: UART to UBlox
#define IRQ_PRIORITY_UART1 0  // VIC1.2: UART to "Debug" port
#define IRQ_PRIORITY_I2C1 4  // VIC1.4: I2C to magnetometer and EEPROM
#define IRQ_PRIORITY_SPI0 3  // VIC1.5: SPI to SPI port
#define IRQ_PRIORITY_SSP1 5  // VIC1.6: SPI to SD Card
#define IRQ_PRIORITY_NEW_DATA 7  // VIC1.11: New data received
#define IRQ_PRIORITY_50HZ 8  // VIC1.12: 50 Hz software interrupt
#define IRQ_PRIORITY_SD_SWITCH 9  // VIC1.13: SD card inserted switch


#endif // _IRQ_PRIORITY_H
