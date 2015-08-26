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
#define IRQ_PRIORITY_SD_SWITCH 14  // VIC1.10: switch at SD card slot switch
#define IRQ_PRIORITY_SW	15  //VIC1.11: Software interrupt


#endif // _IRQ_PRIORITY_H
