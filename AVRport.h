// Include definitions to identify WHAT SPI port is used
// the idea is to be able to support even the SPI MAster included in the UART
// in order to have a dedicated SPI interface to the SD card 

#ifndef AVR_PORT_H
#define AVR_PORT_H

#define AVR_PORT SPI0

#endif //AVR_PORT_H
