// Note: replaced originalk map used in WaveHC with bare minimum for AVR DB
#ifndef ArduinoPins_h
#define ArduinoPins_h

#

// SPI port
#ifdef DB_28_PINS
#   define SS_PIN   PIN_PF1
#else
#   define SS_PIN   PIN_PB5
#endif

#define MOSI_PIN PIN_PA4
#define MISO_PIN PIN_PA5
#define SCK_PIN  PIN_PA6

#endif // ArduinoPins_h
