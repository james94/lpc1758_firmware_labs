/*
 * AudioDecoderVS1053.h
 *
 *  Created on: Apr 25, 2018
 *      Author: james
 */

#ifndef AUDIODECODERVS1053_H_
#define AUDIODECODERVS1053_H_

#include "LPC17xx.h"
#include "tasks.hpp"
#include "gpio.hpp"
#include "ssp0.h"
#include "printf_lib.h"
#include "io.hpp"

/**
 * OpCode to Write to SCI Registers
 */
const uint8_t SCI_WRITE = 0x02;

/**
 * OpCode to Read SCI Registers
 */
const uint8_t SCI_READ = 0x03;

/**
 * VS1053b SCI Registers
 */
const uint8_t SCI_MODE_ADDR = 0x00;//RW
const uint8_t SCI_STATUS_ADDR = 0x01;//RW
const uint8_t SCI_BASS_ADDR = 0x02;//RW
const uint8_t SCI_CLOCKF_ADDR = 0x03;//RW
const uint8_t SCI_DECODE_TIME_ADDR = 0x04;//RW
const uint8_t SCI_AUDATA_ADDR = 0x05;//RW
const uint8_t SCI_WRAM_ADDR = 0x06;//RW
const uint8_t SCI_WRAMADDR_ADDR = 0x07;//W
const uint8_t SCI_HDAT0_ADDR = 0x08;//R
const uint8_t SCI_HDAT1_ADDR = 0x09;//RW
const uint8_t SCI_AIADDR_ADDR = 0x0A;//RW
const uint8_t SCI_VOL_ADDR = 0x0B;//RW
const uint8_t SCI_AICTRL0_ADDR = 0x0C;//RW
const uint8_t SCI_AICTRL1_ADDR = 0x0D;//RW
const uint8_t SCI_AICTRL2_ADDR = 0x0E;//RW
const uint8_t SCI_AICTRL3_ADDR = 0x0F;//RW

/**
 * SCI_MODE Register Bits Set
 */
const uint16_t SET_DEFAULT_SM_SDINEW = 0x4800;//VS1053 native SPI mode
const uint16_t SET_SM_DIFF = 0x0001; //Set Differential
const uint16_t SET_SM_LAYER = 0x0002; //Allow MPEG Layers I & II
const uint16_t SET_SM_RESET = 0x0004; //Set Soft Reset
const uint16_t SET_SM_CANCEL = 0x0008; //Cancel decoding current file
const uint16_t SET_SM_EARSPEAKER_LO = 0x0010; //Set EarSpeaker Low Setting
const uint16_t SET_SM_TESTS = 0x0020; //Allow SDI Tests
const uint16_t SET_SM_STREAM = 0x0040; //Set Stream Mode
const uint16_t SET_SM_EARSPEAKER_HI = 0x0080; //Set EarSpeaker High Setting
const uint16_t SET_SM_DACT = 0x0100; //DCLK Active Edge is Falling Edge
const uint16_t SET_SM_SDIORD = 0x0200; //SDI Bit Order MSb last
const uint16_t SET_SM_SDISHARE = 0x0400; //Set Share SPI Chip Select
const uint16_t SET_SM_SDINEW = 0x0800; //Set VS1053b Native SPI Mode
const uint16_t SET_SM_ADPCM = 0x1000; //Set PCM/ADPCM Recording Active
const uint16_t SET_SM_RESERVED = 0x2000; //WRONG, should be never set
const uint16_t SET_SM_LINE1 = 0x4000; //Set LINE1 Selector
const uint16_t SET_SM_CLK_RANGE = 0x8000; //Set Input Clock Range 24-26MHz

/**
 * SCI_CLOCKF SC_MULT Bits[15:13]:
 * - Multiply XTALI by one of the set multipliers
 * x 1,2,2.5, etc to retrieve CLKI
 */
const uint16_t SET_SC_MULTx1 = 0x0000;
const uint16_t SET_SC_MULTx2 = 0x2000;
const uint16_t SET_SC_MULTx2pt5 = 0x4000;
const uint16_t SET_SC_MULTx3 = 0x6000;
const uint16_t SET_SC_MULTx3pt5 = 0x8000;
const uint16_t SET_SC_MULTx4 = 0xa000;
const uint16_t SET_SC_MULTx4pt5 = 0xc000;
const uint16_t SET_SC_MULTx5 = 0xe000;

/**
 * SCI_CLOCKF SC_ADD Bit[12:11]:
 * - Add set number to multiplier 1,1.5,2
 */
const uint16_t SET_SC_ADD_NO_MOD = 0x0000;
const uint16_t SET_SC_ADD_1_TOMULT = 0x0800; //XTALI*(N+1)
const uint16_t SET_SC_ADD_1pt5_TOMULT = 0x1000; //XTALI*(N+1.5)
const uint16_t SET_SC_ADD_2_TOMULT = 0x1800; //XTALI*(N+2)

/**
 * SCI_VOL Bits[15:0]:
 * - OFF, Low, ..., High, Max
 */
const uint16_t SET_VOLUME_OFF = 0xFEFE;
const uint16_t SET_VOLUME_LOW = 0x5454;
const uint16_t SET_VOLUME_MID = 0x2424;
const uint16_t SET_VOLUME_HIGH = 0x0404;

/**
 * Dummy bytes 2 element array used in initDecoder
 */
extern uint8_t dummy_init[2];

/**
 * Extra Parameters
 */
const uint16_t PARAM_VERSION = 0x0003;

struct common_params {
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
 * 9. Extra Parameter Addresses
 */
const uint16_t PARA_VERSION_ADDR = 0x1e02;
const uint16_t PARA_CONFIG1_ADDR = 0x1e03;
const uint16_t PARA_PLAYSPEED_ADDR = 0x1e04;
const uint16_t PARA_BYTERATE_ADDR = 0x1e05;
const uint16_t PARA_ENDFILLBYTE_ADDR = 0x1e06;
const uint16_t PARA_RESERVED_ADDR0 = 0x1e07;
const uint16_t PARA_RESERVED_ADDR1 = 0x1e08;
const uint16_t PARA_RESERVED_ADDR2 = 0x1e09;
const uint16_t PARA_RESERVED_ADDR3 = 0x1e0A;
const uint16_t PARA_RESERVED_ADDR4 = 0x1e0B;
const uint16_t PARA_RESERVED_ADDR5 = 0x1e0C;
const uint16_t PARA_RESERVED_ADDR6 = 0x1e0D;
const uint16_t PARA_RESERVED_ADDR7 = 0x1e0E;
const uint16_t PARA_RESERVED_ADDR8 = 0x1e0F;
const uint16_t PARA_RESERVED_ADDR9 = 0x1e10;
const uint16_t PARA_RESERVED_ADDR10 = 0x1e11;
const uint16_t PARA_RESERVED_ADDR11 = 0x1e12;
const uint16_t PARA_RESERVED_ADDR12 = 0x1e13;
const uint16_t PARA_RESERVED_ADDR13 = 0x1e14;
const uint16_t PARA_RESERVED_ADDR14 = 0x1e15;
const uint16_t PARA_JUMPPOINTS_ADDR0 = 0x1e16;
const uint16_t PARA_JUMPPOINTS_ADDR1 = 0x1e17;
const uint16_t PARA_JUMPPOINTS_ADDR2 = 0x1e18;
const uint16_t PARA_JUMPPOINTS_ADDR3 = 0x1e19;
const uint16_t PARA_JUMPPOINTS_ADDR4 = 0x1e1A;
const uint16_t PARA_JUMPPOINTS_ADDR5 = 0x1e1B;
const uint16_t PARA_JUMPPOINTS_ADDR6 = 0x1e1C;
const uint16_t PARA_JUMPPOINTS_ADDR7 = 0x1e1D;
const uint16_t PARA_JUMPPOINTS_ADDR8 = 0x1e1E;
const uint16_t PARA_JUMPPOINTS_ADDR9 = 0x1e1F;
const uint16_t PARA_JUMPPOINTS_ADDR10 = 0x1e20;
const uint16_t PARA_JUMPPOINTS_ADDR11 = 0x1e21;
const uint16_t PARA_JUMPPOINTS_ADDR12 = 0x1e22;
const uint16_t PARA_JUMPPOINTS_ADDR13 = 0x1e23;
const uint16_t PARA_JUMPPOINTS_ADDR14 = 0x1e24;
const uint16_t PARA_JUMPPOINTS_ADDR15 = 0x1e25;
const uint16_t PARA_LATESTJUMP_ADDR = 0x1e26;
const uint16_t PARA_POSITIONMSEC_ADDR0 = 0x1e27;
const uint16_t PARA_POSITIONMSEC_ADDR1 = 0x1e28;
const uint16_t PARA_RESYNC_ADDR = 0x1e29;

extern common_params param_data;

/**
 * MP3 Hello World Audio Data
 */
extern uint8_t HelloSampleMP3[];


/**
 * AudioDecoderVS1053 controls the VS1053 chip
 */
class AudioDecoderVS1053 {
private:
    GPIO *m_sci_cs, *m_sdi_cs, *m_dreq, *m_xreset;
    uint16_t m_bass, m_clock, m_vol, m_spi_speed;
public:
    AudioDecoderVS1053();
    AudioDecoderVS1053(GPIO *xcs, GPIO *xdcs, GPIO *dreq, GPIO *xreset);
    virtual ~AudioDecoderVS1053();

    /**
     * Basis Functions for VS1053b decoder:
     * SPI Serial Command Interface (SCI) Methods:
     * - sciWriteReg()
     * - sciReadReg()
     * SPI Serial Data Interface (SDI) Methods:
     * - sdiFillDataBuffer()
     */

    void sciWriteReg(uint8_t reg_addr, uint16_t config);
    uint16_t sciReadReg(uint8_t reg_addr);

    void sdiFillDataBuffer(uint8_t *buffer, uint32_t size);

    /**
     * Interface resets decoder hardware
     */
    void resetHardware();

    /**
     * Interface resets decoder software
     */
    void resetSoftware();

    /**
     * Interface controls volume for decoder hardware
     * MSB controls left channel
     * LSB controls right channel
     *
     * OFF  = 0xFEFE
     * LOW  = 0x5454
     * MID  = 0x2424
     * HIGH = 0x0404
     */
    void setVolume(uint8_t left_channel, uint8_t right_channel);

    /**
     * Initializes Decoder
     */
    bool initDecoder();

    /**
     * Plays a tune
     */
    void sineTest();

    /**
     * Plays Hello World Sample
     */
    void playSample(uint8_t *buffer, uint32_t size);

};

#endif /* AUDIODECODERVS1053_H_ */
