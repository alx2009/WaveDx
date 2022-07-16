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

//* not availble for 28 and 32 pin AVRs


#ifndef AVR_PORT_H
#define AVR_PORT_H

#define AVR_SPI_PORT SPI0

#ifdef DB_28_PINS
#   define AVR_TCA_PORT   TCA0
#   define TCA_OVF_vect TCA0_OVF_vect
#   define TCA_OVF_vect_num 9
#   define TCA_CMP0_vect TCA0_CMP0_vect
#   define TCA_CMP1_vect TCA0_CMP1_vect
    inline void takeOverTCA() {takeOverTCA0();}
    inline void resumeTCA() {resumeTCA0();}
#   define DBG_SERIAL Serial2    
#else
#   define AVR_TCA_PORT   TCA1
#   define TCA_OVF_vect TCA1_OVF_vect
#   define TCA_OVF_vect_num 47
#   define TCA_CMP0_vect TCA1_CMP0_vect
#   define TCA_CMP1_vect TCA1_CMP1_vect
    inline void takeOverTCA() {takeOverTCA1();}
    inline void resumeTCA() {resumeTCA1();}
#   define DBG_SERIAL Serial3    
#endif


#endif //AVR_PORT_H
