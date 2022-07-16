/* Arduino WaveHC Library
 * Copyright (C) 2008 by William Greiman
 *
 * This file is part of the Arduino WaveHC Library
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

 * You should have received a copy of the GNU General Public License
 * along with the Arduino WaveHC Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
#if ARDUINO < 100
#include <WProgram.h>
#else // ARDUINO < 100
#include <Arduino.h>
#endif // ARDUINO < 100
#include <SdReader.h>
#include "AVRport.h"
#include "ArduinoPins.h"
//------------------------------------------------------------------------------
// inline SPI functions
/** Send a byte to the card */
inline void spiSend(uint8_t b) {
  while (!(AVR_USPI_PORT.STATUS & USART_DREIF_bm)) // Make sure the TX buffer is available 
    ;
  AVR_USPI_PORT.STATUS = USART_TXCIF_bm; // clear transmission complete flag
  AVR_USPI_PORT.TXDATAL = b;
  while (!(AVR_USPI_PORT.STATUS & USART_TXCIF_bm)) // wait for transmission complete -  this is safe but it doesn't take advantage of the buffer
    ;
}

/** Read a byte from the receive buffer (can only be used after spiSend) */
uint8_t spiSkip() {
  while (!(AVR_USPI_PORT.STATUS & USART_RXCIF_bm)) // Make sure the RX buffer is available (currently not needed after spiSend but may change in the future)
    ;
  return AVR_USPI_PORT.RXDATAL;  
}

/** Receive a byte from the card */
inline uint8_t spiRec(void) {
  spiSend(0XFF);
  while (!(AVR_USPI_PORT.STATUS & USART_RXCIF_bm)) // Make sure the RX buffer is available (currently not needed after spiSend but may change in the future)
    ;
  return AVR_USPI_PORT.RXDATAL; 
}
/** Set Slave Select high */
inline void spiSSHigh(void) {
  digitalWrite(SS_PIN, HIGH);
  // insure SD data out is high Z
  //spiSend(0XFF);
  spiRec();
}
/** Set Slave Select low */
inline void spiSSLow(void) { digitalWrite(SS_PIN, LOW); }
//------------------------------------------------------------------------------
// card status
/** status for card in the ready state */
#define R1_READY_STATE 0
/** status for card in the idle state */
#define R1_IDLE_STATE 1
/** start data token for read or write */
#define DATA_START_BLOCK 0XFE
/** mask for data response tokens after a write block operation */
#define DATA_RES_MASK 0X1F
/** write data accepted token */
#define DATA_RES_ACCEPTED 0X05
/** write data crc error token */
#define DATA_RES_CRC_ERROR 0X0B
/** write data programming error token */
#define DATA_RES_WRITE_ERROR 0X0D
//------------------------------------------------------------------------------
// send command to card
uint8_t SdReader::cardCommand(uint8_t cmd, uint32_t arg) {
  uint8_t r1;

  // end read if in partialBlockRead mode
  readEnd();

  // select card
  spiSSLow();

  // wait up to 300 ms if busy
  waitNotBusy(300);

  // send command
  spiSend(cmd | 0x40);
  spiSkip();

  // send argument
  for (int8_t s = 24; s >= 0; s -= 8) {
    spiSend(arg >> s);
    spiSkip();
  }

  // send CRC
  uint8_t crc = 0XFF;
  if (cmd == CMD0)
    crc = 0X95; // correct crc for CMD0 with arg 0
  if (cmd == CMD8)
    crc = 0X87; // correct crc for CMD8 with arg 0X1AA
  spiSend(crc); 
  spiSkip();

  // wait for response
  for (uint8_t retry = 0; ((r1 = spiRec()) & 0X80) && retry != 0XFF; retry++)
    ;

  return r1;
}
//------------------------------------------------------------------------------
/**
 * Determine the size of an SD flash memory card.
 * \return The number of 512 byte data blocks in the card
 */
uint32_t SdReader::cardSize(void) {
  csd_t csd;
  if (!readCSD(csd))
    return false;
  if (csd.v1.csd_ver == 0) {
    uint8_t read_bl_len = csd.v1.read_bl_len;
    uint16_t c_size = (csd.v1.c_size_high << 10) | (csd.v1.c_size_mid << 2) |
                      csd.v1.c_size_low;
    uint8_t c_size_mult =
        (csd.v1.c_size_mult_high << 1) | csd.v1.c_size_mult_low;
    return (uint32_t)(c_size + 1) << (c_size_mult + read_bl_len - 7);
  } else if (csd.v2.csd_ver == 1) {
    uint32_t c_size = ((uint32_t)csd.v2.c_size_high << 16) |
                      (csd.v2.c_size_mid << 8) | csd.v2.c_size_low;
    return (c_size + 1) << 10;
  } else {
    error(SD_CARD_ERROR_BAD_CSD);
    return 0;
  }
}
//------------------------------------------------------------------------------
/**
 * Initialize a SD flash memory card.
 *
 * \param[in] slow If \a slow is false (zero) the SPI bus will
 * be initialize at a speed of 8 Mhz.  If \a slow is true (nonzero)
 * the SPI bus will be initialize a speed of 4 Mhz. This may be helpful
 * for some SD cards with Version 1.0 of the Adafruit Wave Shield.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 *
 */
uint8_t SdReader::init(uint8_t slow) {
  //uint8_t ocr[4];  //Note: this paramter was present in WaveHC but not used
  uint8_t r;

  USPI_PORTMUX_REG &= (~USPI_PORTMUX_SET_MASK);  //clear USARTx bits 
  USPI_PORTMUX_REG |= USPI_PORTMUX_SET;          //set   USARTx to the needed value

  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
  pinMode(USPI_MOSI_PIN, OUTPUT);
  pinMode(USPI_MISO_PIN, INPUT);
  pinMode(USPI_SCK_PIN, OUTPUT);

/*
#if SPI_INIT_SLOW
  // Enable SPI, Master, clock rate f_osc/128
  AVR_SPI_PORT.CTRLB = 0x04; (Normal buffer mode, disable slave select, SPI mode 0)
  AVR_SPI_PORT.CTRLA = SPI_ENABLE_bm       // Enable module 
                 | SPI_MASTER_bm       // SPI module in Master mode
                 | SPI_PRESC_DIV128_gc; // prescaler in div16 mode
#else  // SPI_INIT_SLOW
  // Enable SPI, Master, clock rate f_osc/64
  AVR_SPI_PORT.CTRLB = 0x04; //Normal buffer mode, disable slave select, SPI mode 0
  AVR_SPI_PORT.CTRLA = SPI_ENABLE_bm       // Enable module
                 | SPI_MASTER_bm       // SPI module in Master mode
                 | SPI_PRESC_DIV64_gc; // prescaler in div128 mode
#endif // SPI_INIT_SLOW
  //DBG_SERIAL.println("SPI port setup");
*/
  AVR_USPI_PORT.CTRLB = USART_RXEN_bm | USART_TXEN_bm; //Enable the USART and keep the other feratures in CTRLB disabled
# if SPI_INIT_SLOW
  // Enable USART in MAster SPI mode, minimum boud rate (0x0040) * 64 ==> F = 64 * 24 / 2048 = 750 KHz
  AVR_USPI_PORT.BAUDL = 0x00;
  AVR_USPI_PORT.BAUDH = 0x80;  
#else  // SPI_INIT_SLOW
  // Enable SPI, Master, minimum boud rate (0x0040) * 64 ==> F = 64 * 24 / 1024 = 1.5 MHz
  AVR_USPI_PORT.BAUDL = 0x00;
  AVR_USPI_PORT.BAUDH = 0x40;  
#endif // SPI_INIT_SLOW
  AVR_USPI_PORT.CTRLA = 0x00; // disable all interrupts and modes we do not need
  AVR_USPI_PORT.CTRLC = USART_CMODE_MSPI_gc; // put the USART in Host SPI mode. UDORD=UCPHA=0
  AVR_USPI_PORT.STATUS = USART_TXCIF_bm | USART_RXSIF_bm | USART_ISFIF_bm | USART_BDF_bm; // Clear all flags that are not automartically cleared
  DBG_SERIAL.println("USART in MSPI mode setup: done");

  // must supply min of 74 clock cycles with CS high.
  for (uint8_t i = 0; i < 10; i++) 
    spiRec();
  DBG_SERIAL.println("74 clock cycles with CS high: done");
  
  // next two lines prevent re-init hang by cards that were in partial read
  spiSSLow();
  DBG_SERIAL.println("After spiSSLow");
  for (uint16_t i = 0; i <= 512; i++)
    spiRec();
  DBG_SERIAL.println("After dummy spiRec");

  // command to go idle in SPI mode
  for (uint8_t retry = 0;; retry++) {
    if ((r = cardCommand(CMD0, 0)) == R1_IDLE_STATE) {
        DBG_SERIAL.println("R1_IDLE_STATE");
        break;
    }
    if (retry == 30) {
      error(SD_CARD_ERROR_CMD0, r);
      DBG_SERIAL.println("retry == 30!");
      return false;
    }
  }
  DBG_SERIAL.println("SD card is now idle");
  // check SD version
  r = cardCommand(CMD8, 0x1AA);
  DBG_SERIAL.println("After CMD8");
  if (r == R1_IDLE_STATE) {
    for (uint8_t i = 0; i < 4; i++) {
      r = spiRec();
    }
    if (r != 0XAA) {
      error(SD_CARD_ERROR_CMD8_ECHO, r);
      return false;
    }
    type(SD_CARD_TYPE_SD2);
  } else if (r & R1_ILLEGAL_COMMAND) {
    type(SD_CARD_TYPE_SD1);
  } else {
    error(SD_CARD_ERROR_CMD8, r);
  }
  DBG_SERIAL.println("SD version check: done");
  // initialize card and send host supports SDHC if SD2
  for (uint16_t t0 = millis();;) {
    cardCommand(CMD55, 0);
    r = cardCommand(ACMD41, type() == SD_CARD_TYPE_SD2 ? 0X40000000 : 0);
    if (r == R1_READY_STATE)
      break;

    // timeout after 2 seconds
    if (((uint16_t)millis() - t0) > 2000) {
      error(SD_CARD_ERROR_ACMD41);
      return false;
    }
  }
  DBG_SERIAL.println("Ca4rd initialization: done");
  // if SD2 read OCR register to check for SDHC card
  if (type() == SD_CARD_TYPE_SD2) {
    if (cardCommand(CMD58, 0)) {
      error(SD_CARD_ERROR_CMD58);
      return false;
    }
    if ((spiRec() & 0XC0) == 0XC0)
      type(SD_CARD_TYPE_SDHC);

    // discard rest of ocr
    for (uint8_t i = 0; i < 3; i++)
      spiRec();
  }

  // use max SPI frequency unless slow is true
  DBG_SERIAL.println("SPI Port re-initialized (high speed)");
  if (slow) {
     AVR_USPI_PORT.BAUDL = 0x80;
     AVR_USPI_PORT.BAUDH = 0x00;  
  } else {
     AVR_USPI_PORT.BAUDL = 0x40;
     AVR_USPI_PORT.BAUDH = 0x00;     
  }
  spiSSHigh();
  DBG_SERIAL.println("After spiSSHigh");
  return true;
}
//------------------------------------------------------------------------------
/**
 * Read part of a 512 byte block from a SD card.
 *
 * \param[in] block Logical block to be read.
 * \param[in] offset Number of bytes to skip at start of block
 * \param[out] dst Pointer to the location that will receive the data.
 * \param[in] count Number of bytes to read
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * 
 * Assumption: Both transmit and receive buffer are empty when this function is called
 */
uint8_t SdReader::readData(uint32_t block, uint16_t offset, uint8_t *dst,
                           uint16_t count) {
  if (count == 0)
    return true;
  if ((count + offset) > 512) {
    return false;
  }
  if (!inBlock_ || block != block_ || offset < offset_) {
    block_ = block;

    // use address if not SDHC card
    if (type() != SD_CARD_TYPE_SDHC)
      block <<= 9;
    if (cardCommand(CMD17, block)) {
      error(SD_CARD_ERROR_CMD17);
      return false;
    }
    if (!waitStartBlock()) {
      return false;
    }
    offset_ = 0;
    inBlock_ = 1;
  }

  uint8_t dummybyte;
  // start first SPI transfer
  while (!(AVR_USPI_PORT.STATUS & USART_DREIF_bm)) // Make sure the TX buffer is available 
    ;
  AVR_USPI_PORT.TXDATAL = 0XFF;

  for (; offset_ < offset; offset_++) { // TODO: take advantage of the buffer to send quicker...
      while (!(AVR_USPI_PORT.STATUS & USART_DREIF_bm)) // Make sure the TX buffer is available 
        ;
      AVR_USPI_PORT.TXDATAL = 0XFF;
      while (!(AVR_USPI_PORT.STATUS & USART_RXCIF_bm)) // Wait until one character available in the received buffer
        ;
      dummybyte = AVR_USPI_PORT.RXDATAL;   
  }
  //Note: at this point we have sent one character more than those received...

  // transfer data
  uint16_t n = count - 1;
  for (uint16_t i = 0; i < n; i++) {
      while (!(AVR_USPI_PORT.STATUS & USART_RXCIF_bm)) // Wait until one character available in the received buffer
          ;
      dst[i] = AVR_USPI_PORT.RXDATAL;
      //while (!(AVR_USPI_PORT.STATUS & USART_DREIF_bm)) // Make sure the TX buffer is available 
      //    ;
      // We just received a character, since we send max 2 before receiving, there must be place in the TX buffer
      AVR_USPI_PORT.TXDATAL = 0XFF;
  }
  //Note: at this point we are still one character TX ahead of RX...

  // wait for last byte
  while (!(AVR_USPI_PORT.STATUS & USART_RXCIF_bm)) // Wait until one character available in the received buffer
      ;
  dst[n] = AVR_USPI_PORT.RXDATAL;
  offset_ += count;
  if (!partialBlockRead_ || offset_ >= 512)
    readEnd();
  return true;
}
//------------------------------------------------------------------------------
/** Skip remaining data in a block when in partial block read mode. 
 ** Assumption: Both transmit and receive buffer are empty when this function is called
***/

void SdReader::readEnd(void) {
  if (inBlock_) {
    // skip data and crc
    while (!(AVR_USPI_PORT.STATUS & USART_DREIF_bm)) // Make sure the TX buffer is available 
      ;
    AVR_USPI_PORT.TXDATAL = 0XFF;
    uint8_t dummybyte;
    while (offset_++ < 513) {
        while (!(AVR_USPI_PORT.STATUS & USART_DREIF_bm)) // Make sure the TX buffer is available 
          ;
        AVR_USPI_PORT.TXDATAL = 0XFF;
        while (!(AVR_USPI_PORT.STATUS & USART_RXCIF_bm)) // Wait until one character available in the received buffer
          ;
        dummybyte = AVR_USPI_PORT.RXDATAL;   
    }
    // wait for last crc byte
    while (!(AVR_USPI_PORT.STATUS & USART_RXCIF_bm)) // Wait until one character available in the received buffer
       ;
    dummybyte = AVR_USPI_PORT.RXDATAL;   
    spiSSHigh();
    inBlock_ = 0;
  }
}
//------------------------------------------------------------------------------
/** read CID or CSR register */
uint8_t SdReader::readRegister(uint8_t cmd, uint8_t *dst) {
  if (cardCommand(cmd, 0)) {
    error(SD_CARD_ERROR_READ_REG);
    return false;
  }
  if (!waitStartBlock())
    return false;

  // transfer data
  for (uint16_t i = 0; i < 16; i++)
    dst[i] = spiRec();

  spiRec(); // get first crc byte
  spiRec(); // get second crc byte

  spiSSHigh();
  return true;
}
//------------------------------------------------------------------------------
// wait for card to go not busy
uint8_t SdReader::waitNotBusy(uint16_t timeoutMillis) {
  uint16_t t0 = millis();
  while (spiRec() != 0XFF) {
    if (((uint16_t)millis() - t0) > timeoutMillis)
      return false;
  }
  return true;
}
//------------------------------------------------------------------------------
/** Wait for start block token */
uint8_t SdReader::waitStartBlock(void) {
  uint8_t r;
  uint16_t t0 = millis();
  while ((r = spiRec()) == 0XFF) {
    if (((uint16_t)millis() - t0) > SD_READ_TIMEOUT) {
      error(SD_CARD_ERROR_READ_TIMEOUT);
      return false;
    }
  }
  if (r == DATA_START_BLOCK)
    return true;
  error(SD_CARD_ERROR_READ, r);
  return false;
}
