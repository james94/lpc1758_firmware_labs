/*
 * MP3DecoderDemo.h
 *
 *  Created on: May 22, 2018
 *      Author: james
 */

#ifndef MP3DECODERDEMO_H_
#define MP3DECODERDEMO_H_
#include <MP3Player/MP3Drivers/AudioDecoder/SPI/AudioDecoderVS1053.h>

extern TaskHandle_t xMP3DecoderHandle;
extern QueueHandle_t mp3_queue;
extern QueueHandle_t adjust_vol_queue;

extern uint8_t mp3_data[512];

typedef enum DecoderPlayback_st
{
    playMusic = 0,
    pauseMusic = 1,
    volumeInc = 2,
    volumeDec = 3
};

extern DecoderPlayback_st decoder_st;

class MP3DecoderDemo : public scheduler_task {
private:
    //Declare Peripheral Pins for the MP3 Decoder
    GPIO xcs, xdcs, dreq, xreset;
    uint16_t mp3_size = sizeof(mp3_data)/sizeof(mp3_data[0]);
public:
    //Declare AudioDecoder Object to act as an interface
    //for modifying the decoders registers and streaming
    //audio data to it's Audio Buffer
    AudioDecoderVS1053 mp3_decoder;

    MP3DecoderDemo();
    virtual ~MP3DecoderDemo();

    /**
     * Receives MP3 File Data from SD Card via xQueue
     */
    bool receiveMP3DataFromSDCard(uint8_t *mp3_queue_data, uint16_t size);

    /**
     * Plays MP3 File Data
     */
    void playMP3File();
    /**
     * Initializes Decoder after FreeRTOS Starts prior to run()
     */
    bool taskEntry(void);

    /**
     * Plays MP3 File Data that comes in from SD Card
     * Also changes volume of song if user requests it
     */
    bool run(void *p);

};

#endif /* MP3DECODERDEMO_H_ */
