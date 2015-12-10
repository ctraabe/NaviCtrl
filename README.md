University of Tokyo NaviCtrl
--

This is alternative firmware for the MikroKopter NaviCtrl board developed by Chris Raabe at the University of Tokyo.

Status
--

**Not yet plug and play**

**USE AT YOUR OWN RISK**

Requirements
--

* NaviCtrl V2.0 or V2.1
* LSM303D magnetometer (IC5 should be unpoulated)

**This firmware requires the following modifications to the NaviCtrl board**

* A pull-up resistor (4.7K) must be added to pin 5 of the FlightCtrl header
* A pull-down resistor (4.7K) must be added to pin 8 of the Extension III header
* An optional button attached between pins 4 and 8 of the Extension III header

To do
--

- Compass calibration via least-squares ellipsoid
- Waypoint management

Intent
--

This firmware is intended to wholly replace the MikroKopter ecosystem. It may or may not be compatible with the MikroKopter tool in the future, but is not as of now.

The primary goal of the firmware is to provide unrestricted navigation and fast SD card logging.

Build and install
--

This software was developed for use with the Sourcery CodeBench Lite toolchain

Hex files can be uploaded using the Mikrokopter Tool or the firmware uploader (Linux) available at https://github.com/ctraabe/MKProgrammer

Program flow
--

This program is based on a combination of interrupts and polling.

Interrupt priority:

1. Millisecond timer (1 kHz)
2. Flight Ctrl SPI interrupt (<40 kHz)
3. SD card SPI interrupt (<32 kHz)
4. I2C to magnetometer and EEPROM (<40 kHz)
5. UBlox UART interrupt (<1 kHz)
6. "Debug" UART interrupt (<1 kHz)
7. Low-priority data processing interrupt (< 300 times per second)
8. Interrupt from FltCtrl (128 Hz)
9. 50 Hz software interrupt (50 Hz)
10. SD card inserted/ejected (rare)

Main loop:

1. Request the latest magnetometer reading from LSM303D
2. Wait for attitude and status update from Flight Ctrl (SPI)
4. Compute position delta, velocity, and heading correction
5. Send data to Flight Ctrl
6. Log data to SD card
7. Wait for signal from Flight Ctrl and then go to 1.
