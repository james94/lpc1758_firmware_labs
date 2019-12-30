/*
 * MP3PlayerDemo.h
 *
 *  Created on: May 9, 2018
 *      Author: james
 */

#ifndef MP3PLAYERDEMO_H_
#define MP3PLAYERDEMO_H_

#include "io.hpp"
#include "semphr.h"
#include "scheduler_task.hpp"
#include <MP3Player/MP3Drivers/GraphicLCDDisplay/SPI/LCDDisplayNokia5110.h>
#include <MP3Player/FreeRTOS/MP3PlayerDemo/MP3DecoderDemo.h>

//Declare Mutex to guard against SPI1 Bus
extern SemaphoreHandle_t spi1_bus_lock;

class MP3PlayerDemo {
public:
    MP3PlayerDemo();
    virtual ~MP3PlayerDemo();
};



/**
 * FreeRTOS Task that displays an Interactive MP3 Menu of Songs
 */
class MP3DisplayDemo : public scheduler_task
{
private:
    //Declare peripheral pins for the LCD Display
    PWM lcd_backlight;
    GPIO sce, reset, dc;
    //Declare sample tracknames for testing, later will be replaced
    //by actual live MP3 metadata coming in from queue
    const char *track1 = "Track001", *track2 = "Track002";
public:
    //Declare LCDDisplay Object to act as an interface for printing
    //strings onto the display
    LCDDisplayNokia5110 lcd_display;

    //Constructor that initializes the GPIO, PWM, LCDDisplayNokia5110
    //and scheduler_task
    MP3DisplayDemo();

    bool init(void); //Initialize Peripherals for Display Demo

    bool taskEntry(void); //Initial text to display onto screen upon startup

    bool run(void *p); //Checks if switch1 or switch2 is pressed then selects track
};

#endif /* MP3PLAYERDEMO_H_ */
