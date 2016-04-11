# RFM69
RFM69 C library port attempt from this arduino library: https://github.com/LowPowerLab/RFM69
Supports RFM69W, RFM69HW, RFM69CW, RFM69HCW (semtech SX1231, SX1231H)

### To be able to use this library: 
####Please define in your project these platform specific functions defined as extern in RFM69.h:
- `extern void noInterrupts();`             function to disable interrupts
- `extern void interrupts();`               function to enable interrupts  
- `extern void RFM69_SetCSPin(bool);`       function to control the GPIO connected to RFM69 chip select (HIGH or LOW)
- `extern bool RFM69_ReadDIO0Pin(void);`    function to read GPIO connected to RFM69 DIO0 (RFM69 interrupt signalling)
- `extern uint8_t SPI_transfer8(uint8_t);`  function to transfer 1byte on SPI with readback
- `extern void Serialprint(char*);`         function to print to serial port a string
- `extern bool Timeout_IsTimeout1(void);`   function for timeout handling, checks if previously set timeout expired
- `extern void Timeout_SetTimeout1(uint16_t);` function for timeout handling, sets a timeout, parameter is in milliseconds (ms)

####Configure SPI before using library functions
- set SPI CPOL= 0 and CPHA = 0 ( in Motorola/Freescale nomenclature), MSB first
- maximum 10MHz SCK clock according to RFM69 datasheet
