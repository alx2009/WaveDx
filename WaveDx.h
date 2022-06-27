/*!
 * @file WaveDx.h
 *
 * This library is a modified version of WaveHX,  which in itseldf is a 
 * highly modified version of Ladyada's Wave Shield library.
 * We have made many changes that may have introduced bugs.
 *
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef WaveDx_h
#define WaveDx_h
#include "Arduino.h"
#include <FatReader.h>
/**
 * \file
 * WaveDx class
 */
/**
 * If nonzero, optimize the player for contiguous files.  It takes
 * longer to open a file but can play contiguous files at higher rates.
 * Disable if you need minimum latency for open.  Also see open by index.
 */
#define OPTIMIZE_CONTIGUOUS 1
/**
 * If defined, include extra checks in the interrupt vector. This waste 
 * a little bit extra time, and as far as I can se it should not be needed.
 * But it was included in the original WaveHC, so it is kept just in case, 
 * albeit not active by default in WaveDx
 */
//#define INT_VECT_ADD_EXTRA_CHECKS 1
/**
 * Software volume control should be compatible with Ladyada's library.
 * Uses shift to decrease volume by 6 dB per step. See DAC ISR in WaveDx.cpp.
 * Must be set after call to WaveDx::create().
 * Decreases MAX_CLOCK_RATE to 22050.
 */
//#define DVOLUME 0
/**
 * Set behavior for files that exceed MAX_CLOCK_RATE or MAX_BYTE_RATE.
 * If RATE_ERROR_LEVEL = 2, rate too high errors are fatal.
 * If RATE_ERROR_LEVEL = 1, rate too high errors are warnings.
 * If RATE_ERROR_LEVEL = 0, rate too high errors are ignored.
 */
#define RATE_ERROR_LEVEL 2

/**
 * If MONITOR_INTERRUPT_HANDLERS is defined, set a pin at the beginning and end of the 
 * Timer interrupt handlers. 
 * PINs are defined in ArduinoPins.h
 */
#define MONITOR_INTERRUPT_HANDLERS 1
#ifdef MONITOR_INTERRUPT_HANDLERS
#   define MONITOR_PIN_SET(MYVPORT, MYPIN_bm)  MYVPORT.OUT |= MYPIN_bm
#   define MONITOR_PIN_CLR(MYVPORT, MYPIN_bm)  MYVPORT.OUT &= (~MYPIN_bm)
#else
#   define MONITOR_PIN_SET(VPORT, PIN_BM)  
#   define MONITOR_PIN_CLR(VPORT, PIN_BM)  
#endif //MONITOR_INTERRUPT_HANDLERS
//------------------------------------------------------------------------------
// Set the size for wave data buffers.  Must be 256 or 512.
#if defined(__AVR_ATmega168P__) || defined(__AVR_ATmega168__)

/** Buffer length for for 168 Arduino. */
#define PLAYBUFFLEN 256UL
#else // __AVR_ATmega168P__

/** Buffer length for Arduinos other than 168. */
#define PLAYBUFFLEN 512UL
#endif //__AVR_ATmega168P__

// Define max allowed SD read rate in bytes/sec.
#if PLAYBUFFLEN == 512UL && OPTIMIZE_CONTIGUOUS
/** Maximum SD read rate for 512 byte buffer and contiguous file */
#define MAX_BYTE_RATE 88200
#else // MAX_BYTE_RATE
/** Maximum SD read rate for 256 byte buffer or fragmented file */
#define MAX_BYTE_RATE 44100
#endif // MAX_BYTE_RATE

// Define maximum clock rate for DAC.
#ifdef DVOLUME
/** maximum DAC clock rate */
#define MAX_CLOCK_RATE 44100
#else // DVOLUME
/** Decreased clock rate if volume control is used */
#define MAX_CLOCK_RATE 22050
#endif // DVOLUME

//------------------------------------------------------------------------------
/**
 * \class WaveDx
 * \brief Wave file player.
 *
 * Play wave files from FAT16 and FAT32 file systems
 * on SD and SDHC flash memory cards.
 *
 */
class WaveDx {
public: 
  //Moved from WaveDx, this alone shaves 10% from the OVF interrupt execution due to less register used
  uint8_t *buffer1;
  uint8_t *buffer2;
  uint8_t *playend; ///< end position for current buffer
  uint8_t *playpos; ///< position of next sample
  uint8_t *sdbuff;  ///< SD fill buffer
  uint8_t *sdend;   ///< end of data in sd buffer
  uint8_t sdstatus = 0;
public:
  /** Wave file number of channels. Mono = 1, Stereo = 2 */
  uint8_t Channels;
  /** Wave file sample rate. Must be not greater than 44100/sec. */
  uint32_t dwSamplesPerSec;
  /** Wave file bits per sample.  Must be 8 or 16. */
  uint8_t BitsPerSample;
  /** Remaining bytes to be played in Wave file data chunk. */
  uint32_t remainingBytesInChunk;
  /** Has the value true if a wave file is playing else false. */
  volatile uint8_t isplaying;
  /** Number of times data was not available from the SD in the DAC ISR */
  uint32_t errors;

  /** Software volume control. Reduce volume by 6 dB per step. See DAC ISR. */
  uint8_t volume;
  /** FatReader instance for current wave file. */
  FatReader *fd;

  WaveDx(void);
  uint8_t create(FatReader &f);
  /*!
   @brief Return the size of the WAV file
   @returns the size of the WAV file
  */
  uint32_t getSize(void) { return fd->fileSize(); }
  uint8_t isPaused(void);
  void pause(void);
  void play(void);
  int16_t readWaveData(uint8_t *buff, uint16_t len);
  void resume(void);
  void seek(uint32_t pos);
  void setSampleRate(uint32_t samplerate);
  void stop(void);

  void debugPrint();
};

#endif // WaveDx_h
