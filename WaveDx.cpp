/* Arduino WaveDx Library, a port of WaveDx to AVR Dx architecture
 * Original work WaveDx.cpp Copyright (C) 2009 by William Greiman
 * Derived work WaveDx.cpp Copyright (C) 2022 ALX2009
 *
 * This file is part of the Arduino WaveDx Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino WaveDx Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
\mainpage Arduino WaveDx Library
<CENTER>Copyright &copy; 2009 by William Greiman / Copyright &copy; 2022 ALX2009
</CENTER>

\section Intro Introduction

WaveDx is an Arduino library to play sounds using an SD card and the built
in ADC in AVR Dx microcontrollers.  It can play
uncompressed mono Wave(.WAV) files at sample rate up to 44.1 K samples per
second. Only the high 12 bits of 16-bit files are used.  Audio files are read
from an SD flash memory card.

Standard SD and high capacity SDHC flash memory cards are supported with
FAT16 or FAT32 file systems.  The WaveDx only supports short FAT 8.3 names.

WaveDx does not support MMC flash cards.

\section comment Bugs and Comments

If you wish to report bugs or have comments, send email to
fat16lib@sbcglobal.net.

\section SDcard SD/SDHC Cards

Arduinos access SD cards using the cards SPI protocol.  PCs, Macs, and
most consumer devices use the 4-bit parallel SD protocol.  A card that
functions well on A PC or Mac may not work well on the Arduino.

Most cards have good SPI read performance but cards vary widely in
how there SPI hardware interface is implemented.  Newer card require
very fast rise times for SPI signals.  Version 1.0 of the Wave Shield
may not work well with these cards.  Ladyada's improved Version 1.1
works with almost all SD/SDHC cards.

The default SPI clock rate is 8 Mhz.  It may be helpful on Version 1.0
wave shields to reduce this to 4 Mhz. See SdReader::init() for details.

SanDisk cards generally have good performance in the Version 1.0 Wave Shield.


\section WaveDxClass WaveDx Usage

See Ladyada's excellent tutorial on using WaveHC:

http://www.ladyada.net/make/waveshield/libraryhc.html

Also see the readme.txt file for instructions on installing WaveDx.

Advanced users may need to edit the ArduinoPins.h file.

WaveDx uses a slightly restricted form of short file names.
Only printable ASCII characters are supported. No characters with code point
values greater than 127 are allowed.  Space is not allowed even though space
was allowed in the API of early versions of DOS.

Short names are limited to 8 characters followed by an optional period (.)
and extension of up to 3 characters.  The characters may be any combination
of letters and digits.  The following special characters are also allowed:

$ % ' - _ @ ~ ` ! ( ) { } ^ # &

Short names are always converted to upper case and their original case
value is lost.

\section HowTo How to Format and Prepare SD Cards for WaveDx

WaveDx is optimized for contiguous files.  It will only play 16-bit
44.1 K files if they are contiguous.  All files copied to a newly
formatted card will be contiguous.  It is only possible to create
a fragmented file if you delete a file from an SD and copy a larger
file to the SD.

You should use a freshly formatted SD card for best performance.  FAT
file systems become slower if many files have been created and deleted.
This is because the directory entry for a deleted file is marked as deleted,
but is not deleted.  When a file is opened, these entries must be scanned
to find the file to be opened, a flaw in the FAT design.  Also files can
become fragmented which causes reads to be slower.

Microsoft operating systems support removable media formatted with a
Master Boot Record, MBR, or formatted as a super floppy with a FAT Boot Sector
in block zero.

Microsoft operating systems expect MBR formatted removable media
to have only one partition. The first partition should be used.

Microsoft operating systems do not support partitioning SD flash cards.
If you erase an SD card with a program like KillDisk, Most versions of
Windows will format the card as a super floppy.

The best way to restore an SD card's format is to use SDFormatter
which can be downloaded from:

http://www.sdcard.org/consumers/formatter/

SDFormatter aligns flash erase boundaries with file
system structures which reduces write latency and file system overhead.

SDFormatter does not have an option for FAT type so it may format
small cards as FAT12.

After the MBR is restored by SDFormatter you may need to reformat small
cards that have been formatted FAT12 to force the volume type to be FAT16.

If you reformat the SD card with an OS utility, choose a cluster size that
will result in:

4084 < CountOfClusters && CountOfClusters < 65525

The volume will then be FAT16.

If you are formatting an SD card on OS X or Linux, be sure to use the first
partition. Format this partition with a cluster count in above range.

\section  References References

Adafruit Industries:

http://www.adafruit.com/

http://www.ladyada.net/make/waveshield/

The Arduino site:

http://www.arduino.cc/

For more information about FAT file systems see:

http://www.microsoft.com/whdc/system/platform/firmware/fatgen.mspx

For information about using SD cards as SPI devices see:

http://www.sdcard.org/developers/tech/sdcard/pls/Simplified_Physical_Layer_Spec.pdf

The ATmega328 datasheet:

http://www.atmel.com/dyn/resources/prod_documents/doc8161.pdf

 */

#include <WaveDx.h>
#include <WaveUtil.h>
#include <avr/interrupt.h>
//#include <mcpDac.h>
#include <string.h>
#include "AVRport.h"
#include "ArduinoPins.h"

// verify program assumptions
#if PLAYBUFFLEN != 256 && PLAYBUFFLEN != 512
#error PLAYBUFFLEN must be 256 or 512
#endif // PLAYBUFFLEN

WaveDx *playing = 0;
uint8_t buffer1_array[PLAYBUFFLEN];
uint8_t buffer2_array[PLAYBUFFLEN];

// status of sd
#define SD_READY 1    ///< buffer is ready to be played
#define SD_FILLING 2  ///< buffer is being filled from DS
#define SD_END_FILE 3 ///< reached end of file

//
//
//------------------------------------------------------------------------------
// timer interrupt for DAC
ISR(TCA_OVF_vect) {
   MONITOR_PIN_SET(OVF_MONITOR_VPORT, OVF_MONITOR_PIN_bm);
   AVR_TCA_PORT.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;  // Clear flag
# ifdef INT_VECT_ADD_EXTRA_CHECKS
     if (!playing) {
       MONITOR_PIN_CLR(OVF_MONITOR_VPORT, OVF_MONITOR_PIN_bm);
       return;
     }
#endif //INT_VECT_ADD_EXTRA_CHECKS

  if (playing->playpos >= playing->playend) {
    if (playing->sdstatus == SD_READY) {

      // swap double buffers
      playing->playpos = playing->sdbuff;
      playing->playend = playing->sdend;
      playing->sdbuff = playing->sdbuff != playing->buffer1 ? playing->buffer1 : playing->buffer2;

      playing->sdstatus = SD_FILLING;
      AVR_TCA_PORT.SINGLE.INTCTRL |= TCA_SINGLE_CMP0_bm;  // Enable interrupt on compare
    } else if (playing->sdstatus == SD_END_FILE) {
      playing->stop();  
      MONITOR_PIN_CLR(OVF_MONITOR_VPORT, OVF_MONITOR_PIN_bm);
      return;
    } else {
#     ifdef INT_VECT_ADD_EXTRA_CHECKS
         // count overrun error if not at end of file
         if (playing->remainingBytesInChunk) {
            playing->errors++;
         }
#     endif //INT_VECT_ADD_EXTRA_CHECKS
      MONITOR_PIN_CLR(OVF_MONITOR_VPORT, OVF_MONITOR_PIN_bm);
      return;
    }
  }

  uint8_t dh, dl;
  if (playing->BitsPerSample == 16) {

    // 16-bit is signed
    dh = 0X80 ^ playing->playpos[1];
    dl = playing->playpos[0];
    playing->playpos += 2;
  } else {
    // 8-bit is unsigned
    dh = playing->playpos[0];
    dl = 0;
    playing->playpos++;
  }

#ifdef DVOLUME
  uint16_t tmp = (dh << 8) | dl;
  tmp >>= playing->volume;
  dh = tmp >> 8;
  dl = tmp;
#endif // DVOLUME
// The built in DAC is 10 bits. We can only use DH plus the 2 MSB in DL... For best quality and higher efficiency use an external volume control (DVOLUME not recommended)
  DAC0.DATAL = dl & 0xC0;
  DAC0.DATAH = dh; 
  MONITOR_PIN_CLR(OVF_MONITOR_VPORT, OVF_MONITOR_PIN_bm);
}
//------------------------------------------------------------------------------
// this is the interrupt that fills the playbuffer

ISR(TCA_CMP0_vect) {              //TCA COMPARE 0 vector
  MONITOR_PIN_SET(CMP0_MONITOR_VPORT, CMP0_MONITOR_PIN_bm);
  AVR_TCA_PORT.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;  // Clear flag
  // turn off calling interrupt
  //TIMSK1 &= ~_BV(OCIE1B);             //TIMER1 Timer Interrupt Mask Register 1 (TIMSK1), clear OCIE1B
  AVR_TCA_PORT.SINGLE.INTCTRL &= (~(TCA_SINGLE_CMP0_bm)); // Disable interrupt

  if (playing->sdstatus != SD_FILLING) {
     MONITOR_PIN_CLR(CMP0_MONITOR_VPORT, CMP0_MONITOR_PIN_bm);
     return;
  }
  // enable interrupts while reading the SD - note: the periodic interrupt that feed the DAC need to have higher priority for this to work...
  sei();

  int16_t read = playing->readWaveData(playing->sdbuff, PLAYBUFFLEN);

  cli();
  if (read > 0) {
    playing->sdend = playing->sdbuff + read;
    playing->sdstatus = SD_READY;
  } else {
    playing->sdend = playing->sdbuff;
    playing->sdstatus = SD_END_FILE;
  }
  MONITOR_PIN_CLR(CMP0_MONITOR_VPORT, CMP0_MONITOR_PIN_bm);
}
//------------------------------------------------------------------------------
/** create an instance of WaveDx. */
WaveDx::WaveDx(void) { fd = 0; buffer1=buffer1_array; buffer2=buffer2_array; }
//------------------------------------------------------------------------------
/**
 * Read a wave file's metadata and initialize member variables.
 *
 * \param[in] f A open FatReader instance for the wave file.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.  Reasons
 * for failure include I/O error, an invalid wave file or a wave
 *  file with features that WaveDx does not support.
 */
uint8_t WaveDx::create(FatReader &f) {
  // 18 byte buffer
  // can use this since Arduino and RIFF are Little Endian
#ifdef DVOLUME
  if (!DVOLUME) {
    DBG_SERIAL.println("DVOLUME must be set to non-zero in WaveDx.h");
    return false;
  }
#endif //DVOLUME
  union {
    struct {
      char id[4];
      uint32_t size;
      char data[4];
    } riff; // riff chunk
    struct {
      uint16_t compress;
      uint16_t channels;
      uint32_t sampleRate;
      uint32_t bytesPerSecond;
      uint16_t blockAlign;
      uint16_t bitsPerSample;
      uint16_t extraBytes;
    } fmt; // fmt data
  } buf;

#if OPTIMIZE_CONTIGUOUS
  // set optimized read for contiguous files
  f.optimizeContiguous();
#endif // OPTIMIZE_CONTIGUOUS

  // must start with WAVE header
  if (f.read(&buf, 12) != 12 || strncmp(buf.riff.id, "RIFF", 4) ||
      strncmp(buf.riff.data, "WAVE", 4)) {
    return false;
  }

  // next chunk must be fmt
  if (f.read(&buf, 8) != 8 || strncmp(buf.riff.id, "fmt ", 4)) {
    return false;
  }

  // fmt chunk size must be 16 or 18
  uint16_t size = buf.riff.size;
  if (size == 16 || size == 18) {
    if (f.read(&buf, size) != (int16_t)size) {
      return false;
    }
  } else {
    // compressed data - force error
    buf.fmt.compress = 0;
  }

  if (buf.fmt.compress != 1 || (size == 18 && buf.fmt.extraBytes != 0)) {
    putstring_nl("Compression not supported");
    return false;
  }

  Channels = buf.fmt.channels;
  if (Channels > 2) {
    putstring_nl("Not mono/stereo!");
    return false;
  } else if (Channels > 1) {
    putstring_nl(" Warning stereo file!");
  }

  BitsPerSample = buf.fmt.bitsPerSample;
  if (BitsPerSample > 16) {
    putstring_nl("More than 16 bits per sample!");
    return false;
  }

  dwSamplesPerSec = buf.fmt.sampleRate;
  uint32_t clockRate = dwSamplesPerSec * Channels;
  uint32_t byteRate = clockRate * BitsPerSample / 8;

#if RATE_ERROR_LEVEL > 0
  if (clockRate > MAX_CLOCK_RATE || byteRate > MAX_BYTE_RATE) {
    putstring_nl("Sample rate too high!");
    if (RATE_ERROR_LEVEL > 1) {
      return false;
    }
  } else if (byteRate > 44100 && !f.isContiguous()) {
    putstring_nl("High rate fragmented file!");
    if (RATE_ERROR_LEVEL > 1) {
      return false;
    }
  }
#endif // RATE_ERROR_LEVEL > 0

  fd = &f;

  errors = 0;
  isplaying = 0;
  remainingBytesInChunk = 0;

  volume = 0;
  // position to data
  return readWaveData(0, 0) < 0 ? false : true;
}
//------------------------------------------------------------------------------
/*!
 * @brief Returns true if the player is paused else false.
 * @returns true if paused, false otherwise
 */
uint8_t WaveDx::isPaused(void) {
  cli();
  //uint8_t rtn = isplaying && !(TIMSK1 & _BV(OCIE1A)); //TIMER1 
  uint8_t rtn = isplaying && !(AVR_TCA_PORT.SINGLE.INTCTRL & TCA_SINGLE_OVF_bm);
  sei();
  return rtn;
}
//------------------------------------------------------------------------------
/**
 * Pause the player.
 */
void WaveDx::pause(void) {
  cli();
  //TIMSK1 &= ~_BV(OCIE1A); // disable DAC interrupt        //TIMER1 Timer Interrupt Mask Register 1 (TIMSK1), clear OCIE1A
  AVR_TCA_PORT.SINGLE.INTCTRL &= (~(TCA_SINGLE_OVF_bm)); // Disable interrupt
  sei();
  fd->volume()->rawDevice()->readEnd(); // redo any partial read on resume
}

//------------------------------------------------------------------------------
/**
 * Initialize the built in DAC
 *
 */
void avrDACinit() {
    // Disable Digital Input buffer on DAC0 pin as requird by the data sheet
    pinConfigure(PIN_PD6, PIN_ISC_DISABLE);
 
    DACReference(INTERNAL1V024);
    //DACReference(VDD);
    // enable DAC
    DAC0.CTRLA |= (DAC_OUTEN_bm | DAC_ENABLE_bm);  
}

//------------------------------------------------------------------------------
/**
 * Play a wave file.
 *
 * WaveDx::create() must be called before a file can be played.
 *
 * Check the member variable WaveDx::isplaying to monitor the status
 * of the player.
 */
void WaveDx::play(void) {
  // setup the interrupt as necessary

#ifdef MONITOR_INTERRUPT_HANDLERS
  pinMode(OVF_MONITOR_PIN, OUTPUT);
  digitalWrite(OVF_MONITOR_PIN, LOW);
  pinMode(CMP0_MONITOR_PIN, OUTPUT);
  digitalWrite(CMP0_MONITOR_PIN, LOW);
#endif //MONITOR_INTERRUPT_HANDLERS

  int16_t read;

  playing = this;

  // fill the play buffer
  read = readWaveData(buffer1, PLAYBUFFLEN);
  if (read <= 0)
    return;
  playing->playpos = buffer1;
  playing->playend = buffer1 + read;

  // fill the second buffer
  read = readWaveData(buffer2, PLAYBUFFLEN);
  if (read < 0)
    return;
  playing->sdbuff = buffer2;
  playing->sdend = playing->sdbuff + read;
  playing->sdstatus = SD_READY;

  // its official!
  isplaying = 1;

  // Setup mode for DAC ports
  //mcpDacInit();
  avrDACinit();

  //Tave over and reset TCAx 
  takeOverTCA(); //dxCore function - force the core to stop using the timer and reset to the startup configuration
  AVR_TCA_PORT.SINGLE.CTRLA = 0x00; // Make sure the timer is disabled
  AVR_TCA_PORT.SINGLE.CTRLESET = TCA_SINGLE_CMD_RESET_gc | 0x03; // Set CMD to RESET to do a hard reset of the timer
  AVR_TCA_PORT.SINGLE.CTRLD = 0x00; // Make sure we are in single mode
  AVR_TCA_PORT.SINGLE.INTCTRL = 0x00; //Disable all interrupts
  AVR_TCA_PORT.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm | TCA_SINGLE_CMP0_bm;  // Clear interrupt flags

  cli();

  DBG_SERIAL.print(F("old LVL1VEC=")); DBG_SERIAL.println(CPUINT.LVL1VEC);
  CPUINT.LVL1VEC = TCA_OVF_vect_num; // Set the TCA timer OVF vector as high priority 
  DBG_SERIAL.print(F("new LVL1VEC=")); DBG_SERIAL.println(CPUINT.LVL1VEC);
  // Set up TCA
  
  AVR_TCA_PORT.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc; // Normal mode. Disabled: Compare outputs, lock update. 
  AVR_TCA_PORT.SINGLE.EVCTRL &= ~(TCA_SINGLE_CNTEI_bm);    // Disable event counting
  AVR_TCA_PORT.SINGLE.PER = F_CPU / (dwSamplesPerSec * Channels);
  DBG_SERIAL.print("PER="); DBG_SERIAL.println(AVR_TCA_PORT.SINGLE.PER);
  AVR_TCA_PORT.SINGLE.CMP0 = 1;
  AVR_TCA_PORT.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;         // Enable overflow interrupt
  AVR_TCA_PORT.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;  // enable the timer with clock DIV1
  sei();
/*
  // Set up timer one ==> TCA
  // Normal operation - no pwm not connected to pins
  TCCR1A = 0;                                              //TIMER1 Timer/Counter Control Register A = 0
  // no prescaling, CTC mode (TOP = OCR1A, Immediate update, TOV1 flag set on MAX)
  TCCR1B = _BV(WGM12) | _BV(CS10);                         //TIMER1 Timer/Counter Control Register B, clear all except WGM12 and CS10 
  // Sample rate - play stereo interleaved
  OCR1A = F_CPU / (dwSamplesPerSec * Channels);            //TIMER1 Timer/Counter Output Compare Register A
  // SD fill interrupt happens at TCNT1 == 1
  OCR1B = 1;                                               //TIMER1 Timer/Counter Output Compare Register B
  // Enable timer interrupt for DAC ISR
  TIMSK1 |= _BV(OCIE1A);                                   //TIMER1 Timer Interrupt Mask Register 1 (TIMSK1), set OCIE1A
  */
}
//------------------------------------------------------------------------------
/*! Read wave data.
 *
 * @brief Not for use in applications.  Must be public so SD read ISR can access
 * it. Insures SD sectors are aligned with buffers.
 * @param buff pointer to the buffer where the data should be placed
 * @param len the number of bytes to read.
 * @returns the number of bytes that were actually read.
 */
int16_t WaveDx::readWaveData(uint8_t *buff, uint16_t len) {

  if (remainingBytesInChunk == 0) {
    struct {
      char id[4];
      uint32_t size;
    } header;
    while (1) {
      if (fd->read(&header, 8) != 8)
        return -1;
      if (!strncmp(header.id, "data", 4)) {
        remainingBytesInChunk = header.size;
        break;
      }

      // if not "data" then skip it!
      if (!fd->seekCur(header.size)) {
        return -1;
      }
    }
  }

  // make sure buffers are aligned on SD sectors
  uint16_t maxLen = PLAYBUFFLEN - fd->readPosition() % PLAYBUFFLEN;
  if (len > maxLen)
    len = maxLen;

  if (len > remainingBytesInChunk) {
    len = remainingBytesInChunk;
  }

  int16_t ret = fd->read(buff, len);
  if (ret > 0)
    remainingBytesInChunk -= ret;
  return ret;
}
//------------------------------------------------------------------------------
/** Resume a paused player. */
void WaveDx::resume(void) {
  cli();
  // enable DAC interrupt
  if (isplaying)
    //TIMSK1 |= _BV(OCIE1A);  //TIMER1 Timer Interrupt Mask Register 1 (TIMSK1), set OCIE1A
    AVR_TCA_PORT.SINGLE.INTCTRL |= TCA_SINGLE_OVF_bm; // Enable interrupt
  sei();
}
//------------------------------------------------------------------------------
/**
 * Reposition a wave file.
 *
 * \param[in] pos seek will attempt to position the file near \a pos.
 * \a pos is the byte number from the beginning of file.
 */
void WaveDx::seek(uint32_t pos) {
  // make sure buffer fill interrupt doesn't happen
  cli();
  if (fd) {
    pos -= pos % PLAYBUFFLEN;
    if (pos < PLAYBUFFLEN)
      pos = PLAYBUFFLEN; // don't play metadata
    uint32_t maxPos = fd->readPosition() + remainingBytesInChunk;
    if (maxPos > fd->fileSize())
      maxPos = fd->fileSize();
    if (pos > maxPos)
      pos = maxPos;
    if (fd->seekSet(pos)) {
      // assumes a lot about the wave file
      remainingBytesInChunk = maxPos - pos;
    }
  }
  sei();
}
//------------------------------------------------------------------------------
/** Set the player's sample rate.
 *
 * \param[in] samplerate The new sample rate in samples per second.
 * No checks are done on the input parameter.
 * Note: in WaveDx this function must be called AFTER play() and BEFORE stop()
 */
void WaveDx::setSampleRate(uint32_t samplerate) {
  if (samplerate < 500)
    samplerate = 500;
  if (samplerate > 50000)
    samplerate = 50000;
  // from ladayada's library.
  cli();
  while (AVR_TCA_PORT.SINGLE.CNT != 0)     // Synch update 
    ;

  //OCR1A = F_CPU / samplerate;           //TIMER1 Output Compare Register A (OCR1A), set to F_CPU
  AVR_TCA_PORT.SINGLE.PER = F_CPU / samplerate;
  sei();
}
//------------------------------------------------------------------------------
/** Stop the player. */
void WaveDx::stop(void) {
  //TIMSK1 &= ~_BV(OCIE1A); // turn off interrupt    //TIMER1 Timer Interrupt Mask Register 1 (TIMSK1), clear OCIE1A
  AVR_TCA_PORT.SINGLE.INTCTRL &= (~(TCA_SINGLE_OVF_bm)); // Disable interrupt
  playing->isplaying = 0;
  playing = 0;
  resumeTCA();
}

void WaveDx::debugPrint() {
  DBG_SERIAL.print(F("Rem. bytes="));DBG_SERIAL.print(remainingBytesInChunk);
  DBG_SERIAL.print(F(", isplaying="));DBG_SERIAL.print(isplaying);
  DBG_SERIAL.print(F(", errors="));DBG_SERIAL.print(errors);
  DBG_SERIAL.print(F(", sdstatus="));DBG_SERIAL.print(sdstatus);
}
