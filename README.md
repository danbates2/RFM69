# RFM69
RFM69 C library port attempt from this arduino library: https://github.com/LowPowerLab/RFM69
Supports RFM69W, RFM69HW, RFM69CW, RFM69HCW (semtech SX1231, SX1231H)

To be able to use this library please define these function interfaces in RFM69.h:
- noInterrupts()
- interrupts()        
- RFM69_SetCSPin(par) 
- RFM69_ReadDIO0Pin() 
- SPI_transfer8()     
- Serialprint         
