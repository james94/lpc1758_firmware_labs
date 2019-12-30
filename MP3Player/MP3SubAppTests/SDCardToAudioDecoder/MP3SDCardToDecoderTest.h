/*
 * MP3Decoder.h
 *
 *  Created on: Apr 27, 2018
 *      Author: james
 */

#ifndef MP3SDCARDTODECODERTEST_H_
#define MP3SDCARDTODECODERTEST_H_

/**
 * Song Metadata to display when playing song
 */
typedef struct track_metadata
{
    char song_title[30];
    char artist[30];
    char album[30];
    char genre[30];
};


#include "LPC17xx.h"
#include "lpc_pwm.hpp"
#include "tasks.hpp"
#include "gpio.hpp"
#include "ssp0.h"
#include <demo/lab5_SSP/SSPDriver.hpp>
#include <demo/lab2_Gpio/LabGPIO.hpp>
#include <MP3Player/MP3Drivers/GraphicLCDDisplay/SPI/LCDDisplayNokia5110.h>


#include "printf_lib.h"
#include "io.hpp"
#include "eint.h"
#include "storage.hpp"
#include <time.h>
#include <string.h>

/**
 * GPIO Playback demo includes state
 */
typedef enum state
{
    playSong = 0,
    pauseSong = 1,
    volumeUp = 2,
    volumeDown = 3
};

//typedef enum volumeLevel
//{
//    HIGH = 3,
//    MID = 2,
//    LOW = 1,
//    OFF = 0
//};

/**
 * Volume Levels will be included in the decoder
 */
const uint16_t VOLUME_0 = 0xFEFE;
const uint16_t VOLUME_1 = 0xE4E4;
const uint16_t VOLUME_2 = 0xC4C4;
const uint16_t VOLUME_3 = 0xA4A4;
const uint16_t VOLUME_4 = 0x8484;
const uint16_t VOLUME_5 = 0x6464;
const uint16_t VOLUME_6 = 0x4444;
const uint16_t VOLUME_7 = 0x2424;
const uint16_t VOLUME_8 = 0x0404;

/**
 * MP3 Decoder Part, Goes into VS1053bDecoderDemo
 */
extern TaskHandle_t xDecoderHandle;
extern SemaphoreHandle_t PlayPauseSem;
extern SemaphoreHandle_t NextSem ;
extern SemaphoreHandle_t PrevSem ;
extern SemaphoreHandle_t MenuSem ;


//All my code is based on VS1053 Datasheet

//PinTypes:
//DI = Digital Input, CMOS Input Pad
//DO = Digitla Output, CMOS Input Pad
//DO3 = Digitla Output, CMOS Tri-stated Output Pad
//AO = Analog Output
//AI = Analog Input

//XRESET (PinType = DI) = Active LOW Asynchronous Reset, schmitt-trigger input
//DREQ (PinType = DO) = Data Request, input bus
//XDCS/BSYNC (PinType = DI) = Data Chip Select/Byte Sync
//XCS (PinType = DI) = Chip Select Input (Active LOW)
//RX (PinType = DI) = UAR Receive, Connect to IOVDD if not used
//TX (PinType = DO) = UART Transmit
//SCLK (PinType = DI) = Clock for Serial Bus
//SI (PinType = DI) = Serial Input
//SO (PinType = DO3) = Serial Output
//RIGHT (PinType = AO) = Right Channel Output
///GBUF (PinType = AO) = Common Buffer for Headphones, Don't connect to ground
//LEFT (PinType = AO) = Left Channel Output
//LINE2 (PinType = AI) = Line-in 2 (Right Channel)

/**
 * 9. Functional Description (Pg 33):
 * SCI Registers
 */
const uint8_t SCI_MODE = 0x00;//RW
const uint8_t SCI_STATUS = 0x01;//RW
const uint8_t SCI_BASS = 0x02;//RW
const uint8_t SCI_CLOCKF = 0x03;//RW
const uint8_t SCI_DECODE_TIME = 0x04;//RW
const uint8_t SCI_AUDATA = 0x05;//RW
const uint8_t SCI_WRAM = 0x06;//RW
const uint8_t SCI_WRAMADDR = 0x07;//W
const uint8_t SCI_HDAT0 = 0x08;//R
const uint8_t SCI_HDAT1 = 0x09;//RW
const uint8_t SCI_AIADDR = 0x0A;//RW
const uint8_t SCI_VOL = 0x0B;//RW
const uint8_t SCI_AICTRL0 = 0x0C;//RW
const uint8_t SCI_AICTRL1 = 0x0D;//RW
const uint8_t SCI_AICTRL2 = 0x0E;//RW
const uint8_t SCI_AICTRL3 = 0x0F;//RW

//Bits set in SCI_MODE Register (RW)
const uint16_t DEFAULT_MODE_SM_SDINEW = 0x4800;//VS1053 native SPI mode
const uint16_t SM_DIFF = 0x0001; //Set Differential
const uint16_t SM_LAYER = 0x0002; //Allow MPEG Layers I & II
const uint16_t SM_RESET = 0x0004; //Set Soft Reset
const uint16_t SM_CANCEL = 0x0008; //Cancel decoding current file
const uint16_t SM_EARSPEAKER_LO = 0x0010; //Set EarSpeaker Low Setting
const uint16_t SM_TESTS = 0x0020; //Allow SDI Tests
const uint16_t SM_STREAM = 0x0040; //Set Stream Mode
const uint16_t SM_EARSPEAKER_HI = 0x0080; //Set EarSpeaker High Setting
const uint16_t SM_DACT = 0x0100; //DCLK Active Edge is Falling Edge
const uint16_t SM_SDIORD = 0x0200; //SDI Bit Order MSb last
const uint16_t SM_SDISHARE = 0x0400; //Set Share SPI Chip Select
const uint16_t SM_SDINEW = 0x0800; //Set VS1053b Native SPI Mode
const uint16_t SM_ADPCM = 0x1000; //Set PCM/ADPCM Recording Active
const uint16_t SM_RESERVED = 0x2000; //WRONG, should be never set
const uint16_t SM_LINE1 = 0x4000; //Set LINE1 Selector
const uint16_t SM_CLK_RANGE = 0x8000; //Set Input Clock Range 24-26MHz

//Bits set in SCI_CLOCKF Register (RW)
//SC_MULT enables built-in multiplier to multiply XTALI by N to create higher CLKI
const uint16_t SC_MULTx1 = 0x0000;
const uint16_t SC_MULTx2 = 0x2000;
const uint16_t SC_MULTx2pt5 = 0x4000;
const uint16_t SC_MULTx3 = 0x6000;
const uint16_t SC_MULTx3pt5 = 0x8000;
const uint16_t SC_MULTx4 = 0xa000;
const uint16_t SC_MULTx4pt5 = 0xc000;
const uint16_t SC_MULTx5 = 0xe000;

//SC_ADD tells decoder how much to add to the multiplier
const uint16_t SC_ADD_NO_MOD = 0x0000;
const uint16_t SC_ADD_1_TOMULT = 0x0800; //XTALI*(N+1)
const uint16_t SC_ADD_1pt5_TOMULT = 0x1000; //XTALI*(N+1.5)
const uint16_t SC_ADD_2_TOMULT = 0x1800; //XTALI*(N+2)

#define PARAMETRIC_VERSION 0x0003
/**
 * The following structure is in X memory at address 0x1e02
 * and can be used to change extra params or get useful info
 *
 * The fuse-programmed ID is read at startup and copied to the
 * chipID field. If not available, the value is all zeros. The
 * version field can be used to dtrm the layout of the rest of
 * the structure.
 * - playSpeed makes it possible to fast forward songs
 * - byteRate contains the average bitrate in bytes per second for every code
 *      This value can be used to get an estimate of the remaining playtime
 * - endFillByte indicates what byte value to send after file is sent before SM_CANCEL
 * - positionMsec is a field that gives current play position in a file in milliseconds
 *      Currently it is only available in WMA and Ogg Vorbis
 */
struct parametric {
    /* configs aren't cleared between files */
    uint16_t version; /* 1e02 - structure version */
    uint16_t config1; /* 1e03 - ppss RRRR PS mode, SBR mode, Reverb */
    uint16_t playSpeed; /* 1e04 - 0,1 = normal speed, 2 = twice, 3 = times, etc */
    uint16_t byteRate; /* 1e05 average byterate */

    uint16_t endFillByte; /* 1e06 byte value to send after file sent */
    uint16_t reserved[16]; /* 1e07..15 file byte offsets */
    uint32_t jumpPoints[8]; /* 1e16..25 file byte offsets */
    uint16_t latestJump; /* 1e26 index to lastly updated jumpPoint */
    uint32_t positionMsec; /* 1e27-28 play position, if known (WMA, Ogg Vorbis) */
    int16_t resync; /* 1e29 > 0 for automatic m4a, ADIF, WMA resyncs */
};

/**
 * 9. Parameter Addresses
 */
const uint16_t VERSION_ADDR = 0x1e02;
const uint16_t CONFIG1_ADDR = 0x1e03;
const uint16_t PLAYSPEED_ADDR = 0x1e04;
const uint16_t BYTERATE_ADDR = 0x1e05;
const uint16_t ENDFILLBYTE_ADDR = 0x1e06;
const uint16_t RESERVED_ADDR0 = 0x1e07;
const uint16_t RESERVED_ADDR1 = 0x1e08;
const uint16_t RESERVED_ADDR2 = 0x1e09;
const uint16_t RESERVED_ADDR3 = 0x1e0A;
const uint16_t RESERVED_ADDR4 = 0x1e0B;
const uint16_t RESERVED_ADDR5 = 0x1e0C;
const uint16_t RESERVED_ADDR6 = 0x1e0D;
const uint16_t RESERVED_ADDR7 = 0x1e0E;
const uint16_t RESERVED_ADDR8 = 0x1e0F;
const uint16_t RESERVED_ADDR9 = 0x1e10;
const uint16_t RESERVED_ADDR10 = 0x1e11;
const uint16_t RESERVED_ADDR11 = 0x1e12;
const uint16_t RESERVED_ADDR12 = 0x1e13;
const uint16_t RESERVED_ADDR13 = 0x1e14;
const uint16_t RESERVED_ADDR14 = 0x1e15;
const uint16_t JUMPPOINTS_ADDR0 = 0x1e16;
const uint16_t JUMPPOINTS_ADDR1 = 0x1e17;
const uint16_t JUMPPOINTS_ADDR2 = 0x1e18;
const uint16_t JUMPPOINTS_ADDR3 = 0x1e19;
const uint16_t JUMPPOINTS_ADDR4 = 0x1e1A;
const uint16_t JUMPPOINTS_ADDR5 = 0x1e1B;
const uint16_t JUMPPOINTS_ADDR6 = 0x1e1C;
const uint16_t JUMPPOINTS_ADDR7 = 0x1e1D;
const uint16_t JUMPPOINTS_ADDR8 = 0x1e1E;
const uint16_t JUMPPOINTS_ADDR9 = 0x1e1F;
const uint16_t JUMPPOINTS_ADDR10 = 0x1e20;
const uint16_t JUMPPOINTS_ADDR11 = 0x1e21;
const uint16_t JUMPPOINTS_ADDR12 = 0x1e22;
const uint16_t JUMPPOINTS_ADDR13 = 0x1e23;
const uint16_t JUMPPOINTS_ADDR14 = 0x1e24;
const uint16_t JUMPPOINTS_ADDR15 = 0x1e25;
const uint16_t LATESTJUMP_ADDR = 0x1e26;
const uint16_t POSITIONMSEC_ADDR0 = 0x1e27;
const uint16_t POSITIONMSEC_ADDR1 = 0x1e28;
const uint16_t RESYNC_ADDR = 0x1e29;

extern parametric get_data;

/**
 * 7. SPI (SDI/SCI) Native Mode Functions:
 * SCI Command:
 * - writeSciReg()
 * - readSciReg()
 * SDI transfer Data:
 * - sendSDIData()
 */

void writeSciReg(GPIO *sci_cs, GPIO *dreq, uint8_t sci_reg_addr, uint16_t sci_cmd);
uint16_t readSciReg(GPIO *sci_cs, GPIO *dreq, uint8_t sci_reg_addr);
void sendSDIData(GPIO *sdi_cs, GPIO *dreq, uint8_t *buffer, uint32_t size);

/**
 * 10. Decoder Operation Functions:
 * - hardReset()
 * - softReset()
 * - lowPowerMode() <--- Not Done Yet
 * - Play and Decode Functions
 * - SPI Boot
 * - SDI Test Functions
 * -
 */

void hardReset(GPIO *xreset, GPIO *sci_cs, GPIO *sdi_cs, GPIO *dreq);

/**
 * - softReset() - resets the decoder software by setting the sm_reset bit inside the
 * sci_mode register
 */
void softReset(GPIO *sci_cs, GPIO *dreq);

/**
 * Sets Volume Register (SCI_VOL)
 * Controls volume for the player hardware
 * Most Significant Byte of volume register controls the left channel volume,
 * the low part controls the right channel volume. The channel volume sets the attenuation
 * from the maximum volume level in 0.5dB steps. Thus, max volume is 0x0000
 * and total silence is 0xFEFE
 */
void setVolume(GPIO *sci_cs, GPIO *dreq, uint8_t left_channel, uint8_t right_channel);

/**
 * Initialize SJ-One Board pins connected to Decoder
 */
bool initPins(GPIO *xdcs, GPIO *xcs, GPIO *xreset, GPIO *dreq);

/**
 * Initialize Decoder
 */
bool initDecoder(GPIO *xdcs, GPIO *xcs, GPIO *xreset, GPIO *dreq);


/**
 * SDI Test Functions:
 * - sineTest() checks sound of MP3 Decoder
 * other test functions pending
 */
void sineTest(GPIO *xreset, GPIO *sci_cs, GPIO *sdi_cs, GPIO *dreq);

/**
 * Task to perform different SDI tests on the VS1053b Decoder
 */

void vTestMP3Decoder(void *pvParameters);

/**
 * This function will be apart of the FreeRTOS Decoder Demo App
 */
bool receiveMP3DataFromSDCard(uint8_t *audio_data, uint16_t size);

void printTest();

/**
 * Play and Decode Functions:
 * - playWholeFile() plays an entire MP3 file that is received by audio_queue from read audio files task
 * - sci_cs - param used to chip select serial control interface to write/read decoder registers
 * - sdi_cs - param used to chip select serial data interface to send audio data to decoder
 * - dreq - param used to check hardware decoder pin
 * - ssp - param used to call SPI driver transfer function
 */
void playWholeFile(GPIO *sci_cs, GPIO *sdi_cs, GPIO *dreq, uint8_t *audio_filedata, uint16_t length);

/**
 * Task to play mp3 files and listen to the music
 */
void vPlayMP3Music(void *pvParameters);


/**
 * Starts a task to either test the VS1053b Decoder
 * or play MP3 file music
 */
void runMP3Decoder();

/**
 * MP3 Decoder Part End,
 * MP3 SD Card Part Start
 */

typedef struct {
    uint32_t   fsize;          /* File size */
    uint16_t    fdate;          /* Last modified date */
    uint16_t    ftime;          /* Last modified time */
    uint8_t    fattrib;        /* Attribute */
    char   fname[13];      /* Short file name (8.3 format) */
#if _USE_LFN
    char*  lfname;         /* Pointer to the LFN buffer */
    uint32_t    lfsize;         /* Size of LFN buffer in TCHAR */
#endif
} MP3INFO;

typedef union
{
    uint8_t buffer[128];
    struct
    {
        uint8_t header[3];
        uint8_t title[30];
        uint8_t artist[30];
        uint8_t album[30];
        uint8_t year[4];
        uint8_t comment[28];
        uint8_t zero;
        uint8_t track;
        uint8_t genre;
    } __attribute__((packed));
} ID3v1_t;

//takes up 128 bytes of memory to create one of these structures
//To use the structure properties and reduce space usage, utilize pointers and casting
extern ID3v1_t mp3;

void printMP3Data(uint8_t *mp3_data, uint16_t size);

/**
 * readMP3File() - reads an MP3 File
 */
void readMP3FileMetaData();

void sendMP3DataToDecoder(uint8_t *mp3_data);

void readMP3FileData(const char *filename, uint32_t *offset);

void testReadSDCard(void *vParam);

void runSDCard();

/**
 * Control the States of the Decoder:
 * - Play
 * - Pause
 * - Pending: volumeUp, volumeDown
 */
void controlDecoder(uint16_t *change_volume, uint8_t *volume_counter);
void PlayPauseISR();
void NextISR();
void PrevISR();
void MenuISR();
void vControlMP3Player(void *vParam);
void runController();
void runLCDDisplay();
void vMenuSelect(void *param);

#endif /* MP3SDCARDTODECODERTEST_H_ */
