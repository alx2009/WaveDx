// Include definitions to identify WHAT SPI port is used
// the idea is to be able to support even the SPI MAster included in the UART
// in order to have a dedicated SPI interface to the SD card 

//TIMERS AVR128DBxx and dxCore usage:
// TCA0  ==> PWM 
// TCA1* ==> PWM 
// TCB0  ==> Tone
// TCB1  ==> Servo library
// TCB2  ==> millis
// TCB3* 
// TCD0 ==> PWM
// RTC

* not availble for 28 and 32 pin AVRs


#ifndef AVR_PORT_H
#define AVR_PORT_H

#define AVR_SPI_PORT SPI0

#ifdef DB_28_PINS
#   define AVR_TCB_PORT   TCB1
#else
#   define AVR_TCB_PORT   TCB3
#endif


#endif //AVR_PORT_H
