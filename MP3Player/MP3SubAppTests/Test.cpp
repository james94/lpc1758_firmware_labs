/*
 * Test.cpp
 *
 *  Created on: May 13, 2018
 *      Author: james
 */

#include <MP3Player/MP3SubAppTests/Test.h>

QueueHandle_t mp3_data_queue;

char song_title[] = "Black Spiderman";

void sendAudioData(void *vParam)
{


//    for(uint16_t i = 0; i < 16; i++)
//    {
//        if(xQueueSend(mp3_data_queue, &song_title[i], 1000) != pdTRUE)
//        {
//            u0_dbg_printf("Didn't send song title to receiveMP3Data Task\n");
//        }
//        else
//        {
//            u0_dbg_printf("Sent song title to receiveMP3Data Task");
//        }
//    }
    while(1)
    {
        if(xQueueSend(mp3_data_queue, song_title, 1000) != pdTRUE)
        {
            u0_dbg_printf("Didn't send song title to receiveMP3Data Task\n");
        }
        else
        {
            u0_dbg_printf("Sent song title to receiveMP3Data Task");
        }
        vTaskDelay(1000);
    }
}

void receiveAudioData(void *vParam)
{
//    char song_title[16] = {0};

    char title[16] = {0};



//    for(uint16_t i = 0; i < 16; i++)
//    {
//        if(xQueueReceive(mp3_data_queue, &song_title[i], 2000) != pdTRUE)
//        {
//            u0_dbg_printf("Didn't receive song title from sendMP3Data Task\n");
//        }
//        else
//        {
//            u0_dbg_printf("Did receive song title from sendMP3Data Task\n");
//    //        u0_dbg_printf("%s\n", song_title);
//        }
//    }
    while(1)
    {
        if(xQueueReceive(mp3_data_queue, &title, 1000) != pdTRUE)
        {
            u0_dbg_printf("Didn't receive song title from sendMP3Data Task\n");

        }
        else
        {
            u0_dbg_printf("Did receive song title from sendMP3Data Task\n");
            u0_dbg_printf("%s\n", title);
        }
        vTaskDelay(1000);
    }
}

void runSDCardToDecoder()
{

    mp3_data_queue = xQueueCreate(1, sizeof(song_title));

    xTaskCreate(sendAudioData, "sendMP3Data", 1024, NULL, 1, NULL);

    xTaskCreate(receiveAudioData, "receiveMP3Data", 1024, NULL, 1, NULL);

    vTaskStartScheduler();
}
