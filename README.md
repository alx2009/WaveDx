# ALX2009 WaveDx Library

WORK IN PROGRESS
Status (2022-06-19): Supports only AVRxxDB28 and AVRxxDB48. Everything compiles but it has not been tested yet  

This library is a fork of the Adafruit WaveHC library, ported to the AVR Dx architecture and using the built in DAC

You may want to check the original WaveHC documentation for background:
https://github.com/adafruit/WaveHC

Or Ladyada's excellent tutorial for WaveHC:
http://www.ladyada.net/make/waveshield/libraryhc.html

Developers and advanced Arduino users may wish to read the html
documentation starting with html/index.html.

Try daphc.pde in the WaveHC/examples folder.  If you have
problems run the SdReadTest.pde sketch to get more information.


## AVR64DB28 and AVR128DB48

Those are new (as of 2022) AVR microcontroller and the main target for this port. 
For more information on these parts please see https://github.com/SpenceKonde/DxCore 

The built in 10 bit DAC will be used (rather than an external DAC as in WaveHC).  


## SD CARD INIT PROBLEMS

Some SD card are very sensitive to the SPI bus speed for initialization.
Try setting SPI_INIT_SLOW nonzero if you have initialization problems.
To change edit SdReader.h and change the SPI_INIT_SLOW line to:
#define SPI_INIT_SLOW 1


## Hardware

This library will configure the DAC but it will not activate its output. 

It is assumed that the sketch will use one of the OpAmp integrated in the
microcontroller to buffer/amplify the analog signal (as it is done 
in the examples). The opAmp can drive up to about 20 mA, which may be
sufficient for a small headset. Otherwise a suitable audio amplifier should be used.
A low pass filter is also recommended after the DAC (see 

## PREPARING SD CARDS

As the original WaveHC library, WaveDx supports FAT16/FAT32 formats on SD/SDHC cards.  
WaveDx only supports short 8.3 DOS style file names.

WaveDx is optimized to play contiguous files. It will play 16-bit
44.1 K files if they are contiguous.  All files copied to a newly
formatted card will be contiguous. It is only possible to create a
fragmented file if you delete a file from an SD and copy a another
file to the SD.

You should use a freshly formatted SD card for best performance.  FAT
file systems become slower if many files have been created and deleted.
As files become fragmented reads are slower because the overhead to
read the file allocation table increases.  Also the time to open a file
increases.  This is because the directory entry for a deleted file is
marked as deleted, but is not deleted.  When a file is opened, these
entries must be scanned to find the file to be opened, a flaw in the
FAT design.

The best way to format an SD card is to use SDFormatter which can be
downloaded from:

http://www.sdcard.org/consumers/formatter/

SDFormatter aligns flash  boundaries with file system structures which
reduces latency and file system overhead.  SDFormatter does not have an
option for FAT type so it may format small cards as FAT12.


## EXAMPLES

I have included several updates examples for WaveHC in the WaveHC/examples 
folder.  More examples can be downloaded from the Adafruit website:
http://www.ladyada.net/make/waveshield/examples.html

The updated examples are [TODO: update examples]:

`daphc.ino` - plays all .WAV files on an SD.

`SdReadTest.ino` - A sketch to get more information about an SD card.

`PiSpeaker.ino` - A text-to-voice sketch that reads pi.  You need to
                put the files from the piwav folder in example files
                on an SD.
                
`SampleRateHC.ino` - A modified version of the Adafruit example that
                   sets player sample rate by reading analog pin zero.
                   
`SoftVolume.ino` - A modified version of the Adafruit example
                 for software volume control.


`openByIndex.ino` - Shows how to reduce latency between files for
                  applications that must not have a large latency
                  between files.

                  
## CUSTOM SETTINGS

Advanced users may wish to change WaveHC settings.  Most setting are
defines in these files:

WaveHC.h  - Buffer size, contiguous file optimization, enable software
            volume, and error control.

WavePinDefs.h - Change default pin definitions. Save pin 5.

SdReader.h - Change default SPI bus speed.
