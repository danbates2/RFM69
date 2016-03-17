# RFM69
RFM69 C library port attempt from this arduino library: https://github.com/LowPowerLab/RFM69
Supports RFM69W, RFM69HW, RFM69CW, RFM69HCW (semtech SX1231, SX1231H)

To be able to use this library please define these function interfaces in RFM69.h:
- noInterrupts() - function to disable interrupts in your system
- interrupts()   - function to enable interrupts in your system      
- RFM69_SetCSPin(par) - function to control the GPIO tied to RFM69 chip select
- RFM69_ReadDIO0Pin() - function to read GPIO tied to RFM69 DIO0
- SPI_transfer8()     - function to transfer 1byte on SPI with readback
- Serialprint         - function to print to serial port
