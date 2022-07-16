// Note: replaced originalk map used in WaveHC with bare minimum for AVR DB
#ifndef ArduinoPins_h
#define ArduinoPins_h

#

// SPI port
#ifdef DB_28_PINS
#   define SS_PIN   PIN_PA2
#   warning "DB_28_PINS: SS_PIN=PIN_PA2"
#else
#   define SS_PIN   PIN_PB5
#   warning "DB_32_PINS or larger part: SS_PIN=PIN_PB5"
#endif

#define MOSI_PIN PIN_PA4
#define MISO_PIN PIN_PA5
#define SCK_PIN  PIN_PA6

#define USPI_MOSI_PIN PIN_PA4
#define USPI_MISO_PIN PIN_PA5
#define USPI_SCK_PIN  PIN_PA6

#define USPI_PORTMUX_REG PORTMUX.USARTROUTEA
#define USPI_PORTMUX_SET      0x1
#define USPI_PORTMUX_SET_MASK 0x3

#define OVF_MONITOR_PIN  PIN_PC0
#define OVF_MONITOR_VPORT  VPORTC
#define OVF_MONITOR_PIN_bm  0x01

#define CMP0_MONITOR_PIN PIN_PC1
#define CMP0_MONITOR_VPORT VPORTC
#define CMP0_MONITOR_PIN_bm 0x02

#define CMP1_MONITOR_PIN PIN_PC2
#define CMP1_MONITOR_VPORT VPORTC
#define CMP1_MONITOR_PIN_bm 0x04

#endif // ArduinoPins_h
