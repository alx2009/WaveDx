// Note: replaced originalk map used in WaveHC with bare minimum for AVR DB
#ifndef ArduinoPins_h
#define ArduinoPins_h

#

// SPI port
#ifdef DB_28_PINS
#   define SS_PIN   PIN_PF1
#   warning "DB_28_PINS: SS_PIN=PIN_PF1"
#else
#   define SS_PIN   PIN_PB5
#   warning "DB_32_PINS or larger part: SS_PIN=PIN_PB5"
#endif

#define MOSI_PIN PIN_PA4
#define MISO_PIN PIN_PA5
#define SCK_PIN  PIN_PA6

#define OVF_MONITOR_PIN PIN_PC0
#define CMP_MONITOR_PIN PIN_PC1
#define OVF_MONITOR_SET VPORTC.OUT |= 0x01   
#define OVF_MONITOR_CLR VPORTC.OUT &= (~0x01)
#define CMP_MONITOR_SET VPORTC.OUT |= 0x02
#define CMP_MONITOR_CLR VPORTC.OUT &= (~0x02)

#endif // ArduinoPins_h
