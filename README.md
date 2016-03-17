# RFM69
RFM69 C library port attempt from this arduino library: https://github.com/LowPowerLab/RFM69
Supports RFM69W, RFM69HW, RFM69CW, RFM69HCW (semtech SX1231, SX1231H)

To be able to use this library please define these function interfaces in RFM69.h:
#define noInterrupts()      #error "Define function for disabling interrupts"
#define interrupts()        #error "Define function for enabling interrupts"
#define RFM69_SetCSPin(par) #error "Define function for controlling RFM69 CS pin"
#define RFM69_ReadDIO0Pin() #error "Define function for reading RFM69 DIO0 pin"
#define SPI_transfer8()     #error "Define function for SPI transfer 8 bit"
#define Serialprint         #error "Define serial print function"   
