/*
 * blinkLED.cpp
 *
 *  Created on: Jan 30, 2018
 *      Author: james
 */

#include "blinkLED.hpp"

//The application:
//Uses an internal AND external switch to control
//an on-board and external LED
bool toggle_int = false, toggle_ext = false;
bool toggle_easter = false;

/**
 * 1. Reads from the internal OR external GPIO, depends
 *    on input parameter, connected to the switches.
 * 2. If a button is RELEASED then this task sets a
 *    global variable
 */
void vReadSwitch( void *pvParameters )
{
    /* Get Input Parameters */
    uint32_t peripheral = (uint32_t)(pvParameters);

    /* Define Constants Here */
    typedef enum
    {
        PORT0 = 0, //EXT_SW_PORT
        PORT1 = 1, //INT_SW_PORT
        PORT2 = 2 //EXT_SW_PORT
    }PORTS;

    typedef enum
    {
        PIN0 = 0, //EXT_SW0
        PIN1 = 1, //EXT_SW1
        PIN2 = 2, //EXT_SW2
        PIN3 = 3, //EXT_SW3
        PIN9 = 9, //INT_SW0
        PIN10 = 10, //INT_SW1
        PIN14 = 14, //INT_SW2
        PIN15 = 15 //INT_SW3
    }PINS;

    typedef enum
    {
        INPUT = false,
        OUTPUT = true
    }PIN_DIRECTION;

    typedef enum
    {
        RELEASED = false,
        PRESSED = true
    }SWITCH;

    const uint32_t INTERNAL = 0;
    const uint32_t EXTERNAL = 1;

    /* Define Local Variables and Objects */
    LabGPIO Int_SW0(PORT1, PIN9), Ext_SW0(PORT2, PIN0);
    LabGPIO Easter_SW3(PORT1, PIN15);

    /* Initialization Code */
    //Set both pins as input cause switches
    Int_SW0.setDirection(INPUT);
    Ext_SW0.setDirection(INPUT);
    Easter_SW3.setDirection(INPUT);

    while(1)
    {
        u0_dbg_printf("%i\n", peripheral);
        //uart0_puts("While loop");
        //If peripheral is internal is true, then internal switch
        if (peripheral == INTERNAL)
        {
            uart0_puts("Internal Switch");
            if(Int_SW0.getLevel() == RELEASED)
            {
                uart0_puts("INT Switch Released");
                toggle_int = true;
            }
            if(Int_SW0.getLevel() == RELEASED && Easter_SW3.getLevel() == PRESSED)
            {
                toggle_int = false;
                toggle_easter = true;
            }
        }
        //Else if peripheral is external is true, then external switch
        else if(peripheral == EXTERNAL)
        {
            //Checks if button(switch) released, then sets toggle flag
            if(Ext_SW0.getLevel() == RELEASED)
            {
                uart0_puts("Hi Edgar");
                toggle_ext = true;
            }
        }
    }
}

/**
 * 1. Loops and toggles the state of designated LED
 * , specified by input parameter
 * 2. Clears global variable flag
 */
void vControlLED( void *pvParameters )
{
    /* Get Input Parameter */
    uint32_t peripheral = (uint32_t)(pvParameters);
    /* Define Constants Here */
    typedef enum {
        PORT0 = 0, //EXT_LED_PORT
        PORT1 = 1, //INT_LED_PORT
    }PORTS;

    typedef enum
    {
        PIN0 = 0, //LED0
        PIN1 = 1, //LED1
        PIN2 = 2, //
        PIN3 = 3, //
        PIN4 = 4, //LED2
        PIN8 = 8 //LED3
    }PINS;

    typedef enum
    {
        INPUT = false,
        OUTPUT = true
    }PIN_DIRECTION;

    typedef enum
    {
        INTERNAL,
        EXTERNAL,
    }PERPHL_STATUS;

    typedef enum
    {
        ON = false,
        OFF = true
    }LED_STATE;

    typedef enum
    {
        ONE_HUNDRED_MS = 100,
        TWO_HUNDRED_MS = 200,
        THREE_HUNDRED_MS = 300,
        FOUR_HUNDRED_MS = 400,
        FIVE_HUNDRED_MS = 500
    }DELAY_MS;

    /* Define Local Variables and Objects */
    LabGPIO Int_LED0(PORT1, PIN0), Ext_LED0(PORT0, PIN0);
    LabGPIO Easter_LED0(PORT1, PIN0);
    LabGPIO Easter_LED1(PORT1, PIN1);
    LabGPIO Easter_LED2(PORT1, PIN4);
    LabGPIO Easter_LED3(PORT1, PIN8);

    /* Initialization Code */
    Int_LED0.setDirection(OUTPUT);
    Ext_LED0.setDirection(OUTPUT);
    Easter_LED0.setDirection(OUTPUT);
    Easter_LED1.setDirection(OUTPUT);
    Easter_LED2.setDirection(OUTPUT);
    Easter_LED3.setDirection(OUTPUT);

    while(1)
    {
        //if peripheral is internal is true, then internal LED
        if(peripheral == INTERNAL)
        {
            if(toggle_int == true)
            {
                Int_LED0.set(ON);
                vTaskDelay(ONE_HUNDRED_MS);
                Int_LED0.set(OFF);
                toggle_int = false;
            }
            else if(toggle_easter == true)
            {
                //Blink LED0 2 times
                Easter_LED0.set(ON);
                vTaskDelay(ONE_HUNDRED_MS);
                Easter_LED0.set(OFF);
                vTaskDelay(ONE_HUNDRED_MS);

                Easter_LED0.set(ON);
                vTaskDelay(TWO_HUNDRED_MS);
                Easter_LED0.set(OFF);
                vTaskDelay(TWO_HUNDRED_MS);

                //BLINK LED1 2 times
                Easter_LED1.set(ON);
                vTaskDelay(TWO_HUNDRED_MS);
                Easter_LED1.set(OFF);
                vTaskDelay(TWO_HUNDRED_MS);

                Easter_LED1.set(ON);
                vTaskDelay(TWO_HUNDRED_MS);
                Easter_LED1.set(OFF);
                vTaskDelay(TWO_HUNDRED_MS);

                //BLINK LED2 2times
                Easter_LED2.set(ON);
                vTaskDelay(THREE_HUNDRED_MS);
                Easter_LED2.set(OFF);
                vTaskDelay(THREE_HUNDRED_MS);

                Easter_LED2.set(ON);
                vTaskDelay(THREE_HUNDRED_MS);
                Easter_LED2.set(OFF);
                vTaskDelay(THREE_HUNDRED_MS);

                //BLINK LED3 2 times
                Easter_LED3.set(ON);
                vTaskDelay(FOUR_HUNDRED_MS);
                Easter_LED3.set(OFF);
                vTaskDelay(FOUR_HUNDRED_MS);

                Easter_LED3.set(ON);
                vTaskDelay(FOUR_HUNDRED_MS);
                Easter_LED3.set(OFF);
                vTaskDelay(FOUR_HUNDRED_MS);

                //BLINK LED0,LED1,LED2,LED3 in ascending order
                Easter_LED3.set(ON);
                vTaskDelay(ONE_HUNDRED_MS);
                Easter_LED2.set(ON);
                vTaskDelay(TWO_HUNDRED_MS);
                Easter_LED1.set(ON);
                vTaskDelay(THREE_HUNDRED_MS);
                Easter_LED0.set(ON);
                vTaskDelay(FOUR_HUNDRED_MS);
                Easter_LED3.set(OFF);
                Easter_LED2.set(OFF);
                Easter_LED1.set(OFF);
                Easter_LED0.set(OFF);
                vTaskDelay(FOUR_HUNDRED_MS);
                toggle_easter = false;
            }
        }
        //else if peripheral is external is true, then external LED
        else if(peripheral == EXTERNAL)
        {
            if(toggle_ext == true)
            {
                Ext_LED0.set(ON);
                vTaskDelay(ONE_HUNDRED_MS);
                Ext_LED0.set(OFF);
                toggle_ext = false;
            }
        }
    }
}

/**
 * Creates 4 tasks:
 * 1. two vReadSwitch tasks:
 *      1. One for reading "external" switch
 *      2. Other for reading "internal" switch
 * 2. two vControlLED tasks:
 *      1. One for toggling the "internal" LED
 *      2. Other for toggling the "external" LED
 * Pass in parameter to change each task behavior
 */
void vBlinkLED(void)
{
    const uint32_t STACK_SIZE = 256;
    const uint32_t Priority_Low = 1;
//    const uint32_t Priority_Med = 2;
//    const uint32_t Priority_High = 3;

    uint32_t INT = 0;
    uint32_t EXT = 1;




//    typedef struct
//    {
//        uint32_t INT = 0;
//        uint32_t port = 0;
//        uint32_t pin = 1;
//    }INT_SW0;

    xTaskCreate(vReadSwitch, "ReadIntSwitch", STACK_SIZE, (void *) INT, Priority_Low, 0);
    xTaskCreate(vReadSwitch, "ReadExtSwitch", STACK_SIZE, (void *) EXT, Priority_Low, 0);
    xTaskCreate(vControlLED, "ControlIntLED", STACK_SIZE, (void *) INT, Priority_Low, 0);
    xTaskCreate(vControlLED, "ControlExtLED", STACK_SIZE, (void *) EXT, Priority_Low, 0);
    vTaskStartScheduler();
}
