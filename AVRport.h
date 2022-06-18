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
#   define AVR_TCA_PORT   TCA0
#   define TCA_OVF_vect TCA0_OVF_vect
#   define TCA_CMP0_vect TCA0_CMP0_vect
    inline takeOverTCA() {takeOverTCA0();}
    inline resumeTCA() {resumeTCA0();}
#else
#   define AVR_TCA_PORT   TCA1
#   define TCA_OVF_vect TCA1_OVF_vect
#   define TCA_CMP0_vect TCA1_CMP0_vect
    inline takeOverTCA() {takeOverTCA1();}
    inline resumeTCA() {resumeTCA1();}
#endif


#endif //AVR_PORT_H
