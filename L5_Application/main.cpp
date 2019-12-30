/*
 * Author: James Medel
 * Date: 01/28/18
 *
 * Lab Assignment 1: Multiple Tasks
 */
#include <demo/lab1_Multi_Tasks/multi_tasks.hpp>
#include <demo/lab2_Gpio/blinkLED.hpp>
#include <demo/lab3_Eint/intBlinkLED.hpp>
#include <demo/lab4_Adc_Pwm/RGBDemo.hpp>
#include <demo/lab5_SSP/FlashReader.hpp>
#include <demo/lab6_UART/UARTCalcApp.hpp>
#include <demo/lab7_MMA_Tracking/MMATracker.hpp>
#include <demo/lab8_I2C_Slave_Device/I2CSlaveApp.hpp>
#include <demo/lab9_Watchdogs/watchdogMonitor.hpp>
#include <MP3Player/MP3SubAppTests/Display/MP3DisplayTest.h>
#include <MP3Player/MP3SubAppTests/Test.h>
#include <MP3Player/MP3SubAppTests/SDCardToAudioDecoder/MP3SDCardToDecoderTest.h>

//This library file will integrate all the MP3 submodule demos to create
//a fully functional MP3 Player
#include <MP3Player/FreeRTOS/MP3PlayerDemo/MP3PlayerDemo.h>


enum LABS{
    L1_MultiTasks = 1,
    L2_GPIO_blinkLED = 2,
    L3_EINT_blinkLED = 3,
    L4_ADC_PWM_PotToRGB = 4,
    L5_SSP_FlashReader = 5,
    L6_UART_Calc = 6,
    L7_MMA_Tracker = 7,
    L8_I2C_SLAVE_DEVICE = 8,
    L9_WATCHDOGS = 9,
    MP3_SUB_APP_TESTS = 10,
    MP3_PLAYER = 11 // <-- Added MP3DisplayDemo Task to the FreeRTOS Scheduler
};

int main(void)
{

//    uart0_puts("Enter lab to demo:\n");
    int lab = MP3_SUB_APP_TESTS;
    switch(lab)
    {
        case L1_MultiTasks:
            vTaskPrint();
            break;
        case L2_GPIO_blinkLED:
            vBlinkLED();
            break;
        case L3_EINT_blinkLED:
            vIntBlinkLED();
            break;
        case L4_ADC_PWM_PotToRGB:
            playRGBLEDApp();
            break;
        case L5_SSP_FlashReader:
            runFlashReader();
            break;
        case L6_UART_Calc:
            //User supplies operands and operation used for math computation
            runUser();
            //Performs calculation on users operands using the operation
//            runALU();
            break;
        case L7_MMA_Tracker:
            runMMATracker();
            break;
        case L8_I2C_SLAVE_DEVICE:
            //Need to debug and resolve the problem
            runI2CMonitorRegisters();
            break;
        case L9_WATCHDOGS:
            //Testing to see if works
            runWatchdogMonitor();
            break;
        case MP3_SUB_APP_TESTS:
            spi1_bus_lock = xSemaphoreCreateMutex();
            runSDCard();

            runMP3Decoder();

            runController();

            runLCDDisplay();

            vTaskStartScheduler();
            //runDisplay();
            break;
        case MP3_PLAYER:
            /* This type of semaphore uses priority inheritance, so a task 'taking'
             * a semaphore MUST ALWAYS 'give' the semaphore back once the semaphore
             * is no longer required.*/
            mp3_queue = xQueueCreate(1, sizeof(mp3_data));

            adjust_vol_queue = xQueueCreate(1, sizeof(uint16_t));
//            spi1_bus_lock = xSemaphoreCreateMutex();
//
//            scheduler_add_task(new MP3DisplayDemo());
            scheduler_start();
            break;
        default:
            u0_dbg_printf("Invalid Lab Demo Option\n");
            break;
    }

    return -1;
}
