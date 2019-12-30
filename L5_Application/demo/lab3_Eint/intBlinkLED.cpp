/*
 * intBlinkLED.cpp
 *
 *  Created on: Feb 20, 2018
 *      Author: james
 */


// Driver Application

#include "intBlinkLED.hpp"

// Declare our semaphore
SemaphoreHandle_t xSemaPort_Rise, xSemaPort_Fall, xSemaPort_Both;

// Declare a peripheral structure
typedef struct
{
    //volatile tells the compiler not to optimize these attributes
    //, so it doesn't change them
    uint8_t port = 1;
    uint8_t pin = 0;
} peripheral;


/**
 * Since we have a C++ class handle an interrupt, we need to setup a C function
 * delegate to invoke it. So here is the skeleton code that you can reference
 */

/**
 * Unless you design Singleton class, we need a global instance o our class because
 * the asynchronous c_eint3_handler() will need to invoke our C++ class instance callback
 * WARNING: You must use this same instance while testing your main()
 */
LabGPIOInterrupts gpio_intr_instance;

void c_eint3_handler(void)
{
    gpio_intr_instance.handle_interrupt();
}

// callback function for port 2 pin 1 interrupt
void callbackPortRise(void)
{
    u0_dbg_printf("SW rise-edge toggled\n");
    //set the semaphore using fromISR
    xSemaphoreGiveFromISR(xSemaPort_Rise, NULL);
}

void callbackPortFall(void)
{
    u0_dbg_printf("SW fall-edge toggled\n");
    //set semaphore using fromISR
    xSemaphoreGiveFromISR(xSemaPort_Fall, NULL);
}

void callbackPortBoth(void)
{
    u0_dbg_printf("SW one of either edge toggled\n");
    xSemaphoreGiveFromISR(xSemaPort_Both, NULL);
}

void vIntControlLEDRise( void *pvParameters )
{
    /* Get Input Parameters */
    // Sets a pointer to the address of the parameter passed in
    //peripheral LED = *(peripheral*)(pvParameters);

    u0_dbg_printf("Inside vIntControlLED\n");
//    u0_dbg_printf("LED Port %i Pin %i\n", LED.port, LED.pin);
    /* Define Constants Here */
    typedef enum {
        PORT0 = 0, //EXT_LED_PORT
        PORT1 = 1, //INT_LED_PORT
    }PORTS;

    typedef enum
    {
        PIN0 = 0, //LED0
        PIN1 = 1, //LED1
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
        ON = false,
        OFF = true
    }LED_STATE;

    typedef enum
    {
        ONE_HUNDRED_MS = 100,
        TWO_HUNDRED_MS = 200,
        FIVE_HUNDRED_MS = 500
    }DELAY_MS;

    /* Define Local Variables and Objects */
    LabGPIO LED3(PORT1, PIN8), LED0(PORT1, PIN0);
    LabGPIO LED2(PORT1, PIN4), LED1(PORT1, PIN1);
    /* Initialization Code */
    LED0.setDirection(OUTPUT);
    LED1.setDirection(OUTPUT);
    LED2.setDirection(OUTPUT);
    LED3.setDirection(OUTPUT);

    while(1)
    {
            if(xSemaphoreTake(xSemaPort_Rise, 500))
            {
                for(int i = 0; i < 5; ++i)
                {
                    u0_dbg_printf("Blink_LEDs_Rise_Edge_Int\n");
                    vTaskDelay(ONE_HUNDRED_MS);
                    LED0.set(ON);
                    LED1.set(ON);
                    LED2.set(ON);
                    LED3.set(ON);
                    vTaskDelay(ONE_HUNDRED_MS);
                    LED3.set(OFF);
                    LED2.set(OFF);
                    LED1.set(OFF);
                    LED0.set(OFF);
                    vTaskDelay(ONE_HUNDRED_MS);
                }
            }
    }
}

void vIntControlLEDFall( void *pvParameters )
{
    /* Get Input Parameters */
    // Sets a pointer to the address of the parameter passed in
    //peripheral LED = *(peripheral*)(pvParameters);

    u0_dbg_printf("Inside vIntControlLED\n");
//    u0_dbg_printf("LED Port %i Pin %i\n", LED.port, LED.pin);
    /* Define Constants Here */
    typedef enum {
        PORT0 = 0, //EXT_LED_PORT
        PORT1 = 1, //INT_LED_PORT
    }PORTS;

    typedef enum
    {
        PIN0 = 0, //LED0
        PIN1 = 1, //LED1
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
        ON = false,
        OFF = true
    }LED_STATE;

    typedef enum
    {
        ONE_HUNDRED_MS = 100,
        TWO_HUNDRED_MS = 200,
        FIVE_HUNDRED_MS = 500
    }DELAY_MS;

    /* Define Local Variables and Objects */
    LabGPIO LED3(PORT1, PIN8), LED0(PORT1, PIN0);
    LabGPIO LED2(PORT1, PIN4), LED1(PORT1, PIN1);
    /* Initialization Code */
    LED0.setDirection(OUTPUT);
    LED1.setDirection(OUTPUT);
    LED2.setDirection(OUTPUT);
    LED3.setDirection(OUTPUT);

    while(1)
    {
            if(xSemaphoreTake(xSemaPort_Fall, 500))
            {
                for(int i = 0; i < 5; ++i)
                {
                    u0_dbg_printf("Blink_LEDs_Fall_Edge_Int\n");
                    LED3.set(ON);
                    vTaskDelay(ONE_HUNDRED_MS);
                    LED3.set(OFF);
                    vTaskDelay(ONE_HUNDRED_MS);
                    LED2.set(ON);
                    vTaskDelay(ONE_HUNDRED_MS);
                    LED2.set(OFF);
                    vTaskDelay(ONE_HUNDRED_MS);
                    LED1.set(ON);
                    vTaskDelay(ONE_HUNDRED_MS);
                    LED1.set(OFF);
                    vTaskDelay(ONE_HUNDRED_MS);
                    LED0.set(ON);
                    vTaskDelay(ONE_HUNDRED_MS);
                    LED0.set(OFF);
                    vTaskDelay(ONE_HUNDRED_MS);
                }
            }
    }
}

void vIntControlLEDBoth( void *pvParameters )
{
    u0_dbg_printf("Inside vIntControlLED\n");
    /* Define Constants Here */
    typedef enum {
        PORT0 = 0, //EXT_LED_PORT
        PORT1 = 1, //INT_LED_PORT
    }PORTS;

    typedef enum
    {
        PIN0 = 0, //LED0
        PIN1 = 1, //LED1
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
        ON = false,
        OFF = true
    }LED_STATE;

    typedef enum
    {
        ONE_HUNDRED_MS = 100,
        TWO_HUNDRED_MS = 200,
        FIVE_HUNDRED_MS = 500
    }DELAY_MS;

    /* Define Local Variables and Objects */
    LabGPIO LED3(PORT1, PIN8), LED0(PORT1, PIN0);
    LabGPIO LED2(PORT1, PIN4), LED1(PORT1, PIN1);
    /* Initialization Code */
    LED0.setDirection(OUTPUT);
    LED1.setDirection(OUTPUT);
    LED2.setDirection(OUTPUT);
    LED3.setDirection(OUTPUT);

    while(1)
    {
            if(xSemaphoreTake(xSemaPort_Both, 500))
            {
                for(int i = 0; i < 5; ++i)
                {
                    u0_dbg_printf("Blink_LEDs_Both_Edge_Int\n");
                    LED2.set(ON);
                    vTaskDelay(ONE_HUNDRED_MS);
                    LED2.set(OFF);
                    vTaskDelay(ONE_HUNDRED_MS);
                    LED0.set(ON);
                    vTaskDelay(ONE_HUNDRED_MS);
                    LED0.set(OFF);
                    vTaskDelay(ONE_HUNDRED_MS);
                    LED3.set(ON);
                    vTaskDelay(ONE_HUNDRED_MS);
                    LED3.set(OFF);
                    vTaskDelay(ONE_HUNDRED_MS);
                    LED1.set(ON);
                    vTaskDelay(ONE_HUNDRED_MS);
                    LED1.set(OFF);
                }
            }
    }
}

/**
 * vIntBlinkLED() registers C function as callback for EINT3
 * This is cause we cannot register a C++ function as a callback through isr_register()
 *
 * There are workarounds, such as static functions inside a class, but that design
 * is not covered in this assignment
 */
void vIntBlinkLED(void)
{
    //Port 0 Switches
    peripheral SW_P0P0_RE; //Rising
    SW_P0P0_RE.port = 0;
    SW_P0P0_RE.pin = 0;

    peripheral SW_P0P1_FE; //Falling
    SW_P0P1_FE.port = 0;
    SW_P0P1_FE.pin = 1;

    peripheral SW_P0P26_B; //Both
    SW_P0P26_B.port = 0;
    SW_P0P26_B.pin = 26;

    //Port 2 Switches
    peripheral SW_P2P0_RE;
    SW_P2P0_RE.port = 2;
    SW_P2P0_RE.pin =0;

    peripheral SW_P2P1_FE;
    SW_P2P1_FE.port = 2;
    SW_P2P1_FE.pin = 1;

    peripheral SW_P2P7_B;
    SW_P2P7_B.port = 2;
    SW_P2P7_B.pin = 7;


    const uint32_t STACK_SIZE = 2048;
    const uint32_t Priority_Low = 1;

    xSemaPort_Rise = xSemaphoreCreateBinary();
    xSemaPort_Fall = xSemaphoreCreateBinary();
    xSemaPort_Both = xSemaphoreCreateBinary();

    // create our binary semaphore

    // Init things once
    gpio_intr_instance.init();

    //In the driver code, we have an array for PORT 0 or 2 and pin X that handles just RISING edge
    gpio_intr_instance.attachInterruptHandler(SW_P0P0_RE.port, SW_P0P0_RE.pin, callbackPortRise, RISING);

    //In the driver code, we have an array for PORT 0 or 2 and pin X that handles just FALLING edge
    gpio_intr_instance.attachInterruptHandler(SW_P2P1_FE.port, SW_P2P1_FE.pin, callbackPortFall, FALLING);

    //In the driver code, we have an array for PORT 0 or 2 and pin X that handles just BOTH edges
    gpio_intr_instance.attachInterruptHandler(SW_P2P7_B.port, SW_P2P7_B.pin, callbackPortBoth, BOTH);

    // Register my handler function into the vector table at EINT3_IRQn
    isr_register(EINT3_IRQn, c_eint3_handler);

    // Create tasks and test your interrupt handler
    xTaskCreate(vIntControlLEDRise, "ControlLEDRise", STACK_SIZE, NULL, Priority_Low, NULL);
    xTaskCreate(vIntControlLEDFall, "ControlLEDFall", STACK_SIZE, NULL, Priority_Low, NULL);
    xTaskCreate(vIntControlLEDBoth, "ControlLEDBoth", STACK_SIZE, NULL, Priority_Low, NULL);
    vTaskStartScheduler();
}

