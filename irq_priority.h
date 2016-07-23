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
#define IRQ_PRIORITY_UART0 3  // VIC1.0: UART to UBlox
#define IRQ_PRIORITY_UART1 4  // VIC1.2: UART to "Debug" port
#define IRQ_PRIORITY_UART2 7  // VIC1.3: UART2
#define IRQ_PRIORITY_I2C1 2  // VIC1.4: I2C to magnetometer and EEPROM
#define IRQ_PRIORITY_SSP0 0  // VIC1.5: SPI to SPI port
#define IRQ_PRIORITY_SSP1 1  // VIC1.6: SPI to SD Card
#define IRQ_PRIORITY_NEW_DATA 5  // VIC1.10: New data
#define IRQ_PRIORITY_SD_PRESENT 14  // VIC1.11: SD card inserted/ejected
#define IRQ_PRIORITY_FLT_CTRL 6  // VIC1.12: Interrupt from FlightCtrl
#define IRQ_PRIORITY_50HZ 8  // VIC1.13: 50 Hz software interrupt


#endif // _IRQ_PRIORITY_H
