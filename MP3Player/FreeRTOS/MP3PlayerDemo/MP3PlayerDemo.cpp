/*
 * MP3PlayerDemo.cpp
 *
 *  Created on: May 9, 2018
 *      Author: james
 */

#include <MP3Player/FreeRTOS/MP3PlayerDemo/MP3PlayerDemo.h>

SemaphoreHandle_t spi1_bus_lock;

/**
 * MP3PlayerDemo
 */

MP3PlayerDemo::MP3PlayerDemo()
{
    // TODO Auto-generated constructor stub

}

MP3PlayerDemo::~MP3PlayerDemo()
{
    // TODO Auto-generated destructor stub
}

/**
 * MP3 Submodule Tasks that combine to run the MP3 Player Demo
 */
MP3DisplayDemo::MP3DisplayDemo() :
        scheduler_task("mp3_display_demo", 3 * 512, PRIORITY_LOW),
        lcd_backlight(PWM::pwm4, 1000), sce(P2_0), reset(P2_1), dc(P2_2),
        lcd_display(&sce, &dc, &reset, &lcd_backlight)
{
    //run() will be called every 1000ms
    setRunDuration(1000);
}

bool MP3DisplayDemo::init()
{
    //Initialize pins, display's internal registers and display data RAM (screen)
    lcd_display.initDisplay();

    //adjust liquid crystal display sharpness to 40
    lcd_display.setContrast(40);
    return true;
}

bool MP3DisplayDemo::taskEntry()
{
    lcd_display.printString(0, 0, track1, lcd_display.BLACK);
    lcd_display.printString(0, 8, track2, lcd_display.BLACK);

   // lcd_display.printCharacter(0, 0, 'A', lcd_display.BLACK);
    vTaskDelay(3000);
    return true;
}

bool MP3DisplayDemo::run(void *p)
{
    //receive MP3 file metadata from a queue
    track1 = "Hail Mary";
    track2 = "Hipnotize";

    if( spi1_bus_lock != NULL )
    {
        //Mutex created successfully
        //FreeRTOS will sleep task up to 1sec if spi1_bus_lock not available immediately
        //Mutex Semaphore guarantees that only one device can access SPI1 Bus at a time
        if( xSemaphoreTake(spi1_bus_lock, 1000) == pdTRUE )
        {
            u0_dbg_printf("Took spi_bus_lock\n");
            //If switch 1 (P1_9) is pressed, then write the following onto the display
            if(SW.getSwitch(1))
            {
                lcd_display.printString(0, 0, track1, lcd_display.WHITE);
                lcd_display.printString(0, 8, track2, lcd_display.BLACK);
            }
            else if(SW.getSwitch(2))//else switch 2(P1_10), do the following
            {
                lcd_display.printString(0, 8, track2, lcd_display.WHITE);
                lcd_display.printString(0, 0, track1, lcd_display.BLACK);
            }

            //We've finished accessing the guarded resource, releasing semaphore
            xSemaphoreGive(spi1_bus_lock);
        }
        else
        {
            u0_dbg_printf("MP3DisplayDemo Task not able to access guarded resource.\n");
        }
    }

    return true;
}


