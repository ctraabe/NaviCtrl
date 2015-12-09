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
