/*
 * MP3DecoderDemo.cpp
 *
 *  Created on: May 22, 2018
 *      Author: james
 */

#include <MP3Player/FreeRTOS/MP3PlayerDemo/MP3DecoderDemo.h>


QueueHandle_t mp3_queue;
QueueHandle_t adjust_vol_queue;
DecoderPlayback_st decoder_st = playMusic;

uint8_t mp3_data[512] = {0};

MP3DecoderDemo::MP3DecoderDemo() :
        scheduler_task("mp3_decoder_demo", 3*512, PRIORITY_LOW),
        xcs(P0_30), xdcs(P0_29), dreq(P0_1), xreset(P1_22),
        mp3_decoder(&xcs, &xdcs, &dreq, &xreset)
{
    // Set Run() function to loop every 100ms
    setRunDuration(100);
}

MP3DecoderDemo::~MP3DecoderDemo()
{
    // TODO Auto-generated destructor stub
}

/**
 * Receives MP3 File Data from SD Card via xQueue
 */
bool MP3DecoderDemo::receiveMP3DataFromSDCard(uint8_t *mp3_queue_data, uint16_t size)
{
    bool data_received = false;
    if(xQueueReceive(mp3_queue, mp3_queue_data, 100) != pdTRUE)
    {
        data_received = false;
    }
    else
    {
//        for(uint16_t i = 0; i < size; i++)
//        {
//            u0_dbg_printf("\n0x%x\n", *(audio_data + i));
//        }
        data_received = true;
    }
    return data_received;
}

/**
 * Plays MP3 File Data
 */
void MP3DecoderDemo::playMP3File()
{
    //Default Playback Mode
    uint8_t end_fill_byte = 0;
    bool is_sm_cancel_set = false;
    uint16_t sci_mode_data = 0, sci_hdat0_data = 0, sci_hdat1_data = 0;
    uint16_t bytes_sent = 1;
    bool is_hdat0_cleared = false, is_hdat1_cleared = false;
    uint16_t adjust_volume = 0xFEFE;

    do{
        //1. Send an audio file to VS1053b
        while(receiveMP3DataFromSDCard(mp3_data, mp3_size))
        {
            if(decoder_st == volumeInc || decoder_st == volumeDec)
            {
                if(xQueueReceive(adjust_vol_queue, &adjust_volume, 5) != pdTRUE)
                {
                    u0_dbg_printf("\nFailed to receive desired volume\n");
                }
                else
                {
                    u0_dbg_printf("\nDesired Volume 0x%x\n", adjust_volume);
                    mp3_decoder.sciWriteReg(SCI_VOL_ADDR, adjust_volume);
                }
            }

            //Feed 512 bytes of data chunks to VS1053b in 32 byte chunks until song sent
            mp3_decoder.sdiFillDataBuffer(mp3_data, mp3_size);
        }

        //2. Read extra parameter value endFillByte
        mp3_decoder.sciWriteReg(SCI_WRAMADDR_ADDR, PARA_ENDFILLBYTE_ADDR);
        param_data.endFillByte = mp3_decoder.sciReadReg(SCI_WRAM_ADDR);
        end_fill_byte = param_data.endFillByte & 0xFF;

        //3. Send at least 2052 bytes of endFillByte[7:0]
        for(uint16_t i = 0; i < 2052; i++)
        {
            mp3_decoder.sdiFillDataBuffer(&end_fill_byte, sizeof(end_fill_byte));
        }

        //4. Set SCI_MODE bit SM_CANCEL
        mp3_decoder.sciWriteReg(SCI_MODE_ADDR, SET_DEFAULT_SM_SDINEW | SET_SM_CANCEL);

        do{
            //5. Send at least 32 bytes of endFillByte[7:0]
            for(uint16_t i = 0; i < 32; i++)
            {
                mp3_decoder.sdiFillDataBuffer(&end_fill_byte, sizeof(end_fill_byte));
                bytes_sent++;
            }
            //6. Read SCI_MODE. //If SM_CANCEL is still set, go to 5.
            sci_mode_data = mp3_decoder.sciReadReg(SCI_MODE_ADDR);
            is_sm_cancel_set = sci_mode_data & SET_SM_CANCEL;

            //If SM_CANCEL hasn't cleared after sending 2048 bytes
            if(bytes_sent > 2047)
            {
                mp3_decoder.resetSoftware(); //do a software reset(rare)
            }
        }while(is_sm_cancel_set);

        //7. The song has now been successfully sent.
        //Does HDAT0 and HDAT1 contain 0 meaning no format being decoded?
        sci_hdat0_data = mp3_decoder.sciReadReg(SCI_HDAT0_ADDR);
        is_hdat0_cleared = sci_hdat0_data == 0; //if HDAT0 reg contains zero, return true

        sci_hdat1_data = mp3_decoder.sciReadReg(SCI_HDAT1_ADDR);
        is_hdat1_cleared = sci_hdat1_data == 0; //if HDAT1 reg contains zero, return true
    }while(is_hdat0_cleared && is_hdat1_cleared);

}


bool MP3DecoderDemo::taskEntry(void)
{
    //Initialize Decoder Right After FreeRTOS
    mp3_decoder.initDecoder();
    return true;
}

/**
 * Plays MP3 File Data that comes in from SD Card
 * Also changes volume of song if user requests it
 *
 * Loops at a rate specified by setRunDuration(RATE_MS);
 */
bool MP3DecoderDemo::run(void *p)
{
    playMP3File();
    return true;
}
