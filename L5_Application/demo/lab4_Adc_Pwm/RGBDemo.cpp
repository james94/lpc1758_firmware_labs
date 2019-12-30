/*
 * RGBDemo.cpp
 *
 *  Created on: Feb 25, 2018
 *      Author: james
 */

#include "RGBDemo.hpp"

/**
 * Requirements:
 * 1. Use own ADC Driver to read voltage from Potentiometer, (Done)
 *      print 1 s
 * 2. Use own PWM Driver to drive RGB LED,                   (Not Done)
 *      print duty cycle all 3 pins 1 s
 *
 * 3. For combining both above,
 *      RGB LED must be dependent on ADC input from POT
 * 4. By varying POT, should see changes in RGB LED color
 */

/* Code for Testing Reading Potentiometer Voltage */
/* Testing Completed: 2/25/18 */
void testAnalogLight(void *params)
{
    /*Local Variables or Objects*/
    ADCDriver analogLight;
    float lightVoltage = 0;

    /*Init ADC 0.2 Peripheral Device (Pin 0.25, Channel 2)*/
    analogLight.adcInitBurstMode();
    analogLight.adcSelectPin(analogLight.ADC_PIN_0_25);

    while(1)
    {
        lightVoltage = analogLight.readADCVoltageByChannel(analogLight.ADC_CHANNEL_2);
        u0_dbg_printf("Light Voltage: %0.2f\n", lightVoltage);
    }
}

/*Read voltage from Potentiometer*/
void readPot(void *params)
{
    /*Local Variables or Objects*/
    ADCDriver potentiometer;
    float potVoltage = 0;

    /*Init ADC 0.3 Peripheral (Pin 0.26, Channel 3) */
    potentiometer.adcInitBurstMode();
    potentiometer.adcSelectPin(potentiometer.ADC_PIN_0_26);

    while(1)
    {
        potVoltage = potentiometer.readADCVoltageByChannel(potentiometer.ADC_CHANNEL_3);
        vTaskDelay(1000); //wait 1s then print
        u0_dbg_printf("Pot Voltage: %0.2f\n", potVoltage);
    }
}

void printAnalogLight(void)
{
    u0_dbg_printf("Inside printAnalogLight\n");
    const uint32_t STACK_SIZE = 512;
    const uint32_t Priority_Low = 1;

    xTaskCreate(testAnalogLight, "TestAnalogLight", STACK_SIZE, NULL, Priority_Low, NULL);
}

//1. Able to use my own ADC Driver
//to Read Potentiometer voltage every 1s
void printPotVoltage(void)
{
    u0_dbg_printf("Inside printPotVoltage\n");
    const uint32_t STACK_SIZE = 512;
    const uint32_t Priority_Low = 1;

    xTaskCreate(readPot, "readPot", STACK_SIZE, NULL, Priority_Low, NULL);
}

/*Application to test if PWM Driver works, I have layed out
two test scenarios and the rules of Single Edge PWM:*/

/**
 * Test Scenario 1: Duty Cycle Percentage 50% combined with PWM Freq 1Hz
 * will blink LED once a second. 1Hz signal repeats every 1 sec.
 *
 * Test Scenario 2: Duty Cycle Percentage 50% combined with PWM Freq 1KHz
 * will dim LED to 50% of total capable brightness. 1KHz signal
 * repeats every 1 millisec.
 *
 * Single Edge PWM is controlled by a dedicated MR reg
 *
 * "Rules of Single Edge Controlled PWM"
 *      1. All Single Edge Controlled PWM outputs [PWM1.1-1.6] go High at start of PWM Cycle
 *          "EXCEPT" when their Match Value in MR[1-6]=0
 *          What does that mean? MR[1-6] is value that will reset the
 *          Single Edge PWM cycle for certain channel
 *          Note: Waveform in Fig 121 helped me in understanding
 *          how to write the user test application and the driver functions
 *          the user calls upon.
 *      2. Each PWM output will go low when its match value is reached.
 *          What does that mean?
 *              - If MR[1-6] is set to a value, once the TC counts to that
 *              value, then MR will clear the PWM channel signal to LOW.
 *              - TC will start continue counting until it reaches the "max count"
 *              - Then all PWM channels will be reset to their starting signal
 *              by MR0
 *         If no match occurs, then the output remains continuously high.
 *              This means that MR[1-6] is set to 100 duty cycle percentage.
 */

/* Test Scenario 1: Check if PWM[1-6] makes LEDs blink*/
/* Testing Completed: 2/28/18 */
void vTestPWMChBlinkLED(void *params)
{
    u0_dbg_printf("Entered vTestPWMChBlinkLED()\n");
    /*Local Variables or Objects*/
    PWMDriver pwm1;
    uint32_t pwm1_6_duty_cycle = 0;

    /*Init PWM Peripheral*/
    pwm1.pwmInitSingleEdgeMode(1); //Set PWM Freq to 1Hz will cause LED to Blink
    //pwm1.pwmSelectPin(pwm1.PWM_PIN_2_0); //PWM1.1
    //pwm1.pwmSelectPin(pwm1.PWM_PIN_2_1); //PWM1.2
//    pwm1.pwmSelectPin(pwm1.PWM_PIN_2_2); //PWM1.3
//    pwm1.pwmSelectPin(pwm1.PWM_PIN_2_3); //PWM1.4
//    pwm1.pwmSelectPin(pwm1.PWM_PIN_2_4); //PWM1.5
    pwm1.pwmSelectPin(pwm1.PWM_PIN_2_5); //PWM1.6

    while(1)
    {
        //pwm1.setDutyCycle(pwm1.PWM_PIN_2_0, 100); Set PWM1.1 Duty Cycle to 100%, which means it high always, so LED is OFF
        pwm1.setDutyCycle(pwm1.PWM_PIN_2_5, 50); //Set PWM1.1 Duty Cycle to 0%, which eans it is low always, so LED ON
        //I should see LED blink cause Initially I set the PWM Pin 2.0 to 50%
        pwm1_6_duty_cycle = pwm1.getDutyCycle(pwm1.PWM_PIN_2_5);
        u0_dbg_printf("PWM1.6 P2.5 Duty Cycle: %i\n", pwm1_6_duty_cycle);
        //print duty cycle would be printing the MR value?
        /*Read all 3 PWM Duty Cycles for all three PWM pin*/
        vTaskDelay(1000);
    }
}

void vTestPWMChDimLED(void *params)
{
    u0_dbg_printf("Entered vTestPWMChDimLED()\n");
    /*Local Variables or Objects*/
    PWMDriver pwm1;
    uint32_t pwm1_1_duty_cycle = 0;

    /*Init PWM Peripheral*/
    pwm1.pwmInitSingleEdgeMode(1000); //Set PWM Freq to 1Hz will cause LED to Dim
    pwm1.pwmSelectPin(pwm1.PWM_PIN_2_0); /*PWM1.1*/

    while(1)
    {   //User could potentially pass in how low they want to DIM the LED
        //pwm1.setDutyCycle(pwm1.PWM_PIN_2_0, 100); //Set PWM1.1 Duty Cycle to 100%, so active low LED is DIM at 100% capacity
        //pwm1.setDutyCycle(pwm1.PWM_PIN_2_0, 75); //SetPWM1.1 Duty Cycle to 75%, so active low LED is DIM at 75% capacity
        pwm1.setDutyCycle(pwm1.PWM_PIN_2_0, 50); //Set PWM1.1 Duty Cycle to 50%, so active low LED is DIM at 50% capacity
        //pwm1.setDutyCycle(pwm1.PWM_PIN_2_0, 25); //Set PWM1.1 Duty Cycle to 25%, so active low LED is DIM at 25% capacity
        //pwm1.setDutyCycle(pwm1.PWM_PIN_2_0, 0); //Set PWM1.1 Duty Cycle to 0%, so active low LED is DIM at 0% capacity
        //I should see LED blink cause Initially I set the PWM Pin 2.0 to 50%
        pwm1_1_duty_cycle = pwm1.getDutyCycle(pwm1.PWM_PIN_2_0);
        u0_dbg_printf("PWM1.1 P2.0 Duty Cycle: %i\n", pwm1_1_duty_cycle);
        //print duty cycle would be printing the MR value?
        /*Read all 3 PWM Duty Cycles for all three PWM pin*/
        vTaskDelay(1000);
    }
}

/*Print Duty Cycle of PWM1.1 P2.0 every 1sec*/
void printDutyCycle(void)
{
    u0_dbg_printf("Entered printDutyCycle\n");
    const uint32_t STACK_SIZE = 2048;
    const uint32_t Priority_Low = 1;

    xTaskCreate(vTestPWMChBlinkLED, "testPWMBlinkLED", STACK_SIZE, NULL, Priority_Low, NULL);
    //xTaskCreate(vTestPWMChDimLED, "testPWMDimLED", STACK_SIZE, NULL, Priority_Low, NULL);
}


//All code before this step is test code to verify both drivers
//can be placed inside a new file

/*MVP Application: Interface Potentiometer with an RGB LED*/
/* Drive an RGB LED using PWM Driver*/
/* Two Tasks:
   1. vReadPot to read potentiometer, then send the data via queue
       Note: Print pot volt reading every 1s
   2. vDriveRGBLED to change colors of RGB LED by receiving Pot voltage range
       Note: Print Duty cycle 3 RGB pins every 1s
   3. Data is shared via global QueueHandle_t xQueue
   Hence, the PWM output to RGB LED is dependent on ADC input from Pot
   Outcome: See changes in RGB LED color by varying Pot

   How to Play RGB Change Color:
   1. In the playRGBLEDApp() function, the supported colors you can
   pass into the vDriveRGBLED() are RED, GREEN, BLUE, PURPLE and WHITE.
   2. Type the name of the color to the task you want to see the RGB LED
   illuminate to
   3. Build the App.
   4. Then rotate the potetiometer right to illuminate the RGB LED to the
   desired color.
*/

QueueHandle_t xQueue;

void vReadPotentiometer(void *params)
{
    /*Declare local variables &objects*/
    ADCDriver potentiometer;
    float pot_data_ch3 = 0;
    float *pot_addr_data_ch3;

    /*Create tx queue able to hold 1 float value, float
    is the size of each item, 1 is the amount of items
    the queue will hold */
    xQueue = xQueueCreate( 1, sizeof( float ) );

    /*Init ADC Peripheral*/
    potentiometer.adcInitBurstMode();
    potentiometer.adcSelectPin(potentiometer.ADC_PIN_0_26); /*ADC 0.3*/

    while(1)
    {
        pot_data_ch3 = potentiometer.readADCVoltageByChannel(potentiometer.ADC_CHANNEL_3);
        //Make sure txQueue was created successfully
        if( xQueue != NULL )
        {
            u0_dbg_printf("txQueue was created successfully.\n");
            /* Send pot voltage, if space not available right away
             , then wait 500ms, else print failed to send data.*/
            pot_addr_data_ch3 = &pot_data_ch3;
            if( xQueueSend( xQueue, &pot_addr_data_ch3, 500 ) != pdTRUE)
            {
                u0_dbg_printf("Failed to post data, after 500ms\n");
            }
            else
            {
                u0_dbg_printf("Placed Pot Ch3 Data: %0.2f onto queue\n", *pot_addr_data_ch3);
            }
        }
        vTaskDelay(1000);
    }
}

void vDriveRGBLED(void *params)
{
    /*Get Input for Color User Wants to set RGB LED*/
    uint32_t RGB_COLOR = (uint32_t)(params);

    typedef enum
    {
        RED = 0,
        GREEN = 1,
        BLUE = 2,
        PURPLE = 3,
        BROWN = 4,
        WHITE = 5,
    }RGB_LED_COLOR;

    /*Declare local variables &objects*/
    PWMDriver pwm1_rgb;

    uint32_t pwm_red_dc = 0; //PWM1.1 : Red duty cycle
    uint32_t pwm_green_dc = 0; //PWM 1.2: Green
    uint32_t pwm_blue_dc = 0; //PWM 1.3: Blue

    float *rx_pot_addr_data;

    float pot_volt_percent = 0; //pot volt range is 0mV to 3,300mV

    /*Init PWM Peripheral*/
    pwm1_rgb.pwmInitSingleEdgeMode(1); //Set PWM1 Freq to 1KHz

    //pwm1_rgb.pwmSelectAllPins();
    pwm1_rgb.pwmSelectPin(pwm1_rgb.PWM_PIN_2_0);
    pwm1_rgb.pwmSelectPin(pwm1_rgb.PWM_PIN_2_1);
    pwm1_rgb.pwmSelectPin(pwm1_rgb.PWM_PIN_2_2);

    while(1)
    {
        if( xQueue != NULL)
        {
            u0_dbg_printf("rxQueue was created successfully.\n");
            //Receive pot_data on queue. Wait for 500ms if data isn't
            //immediately.
            if( xQueueReceive( xQueue, &( rx_pot_addr_data ), 500) != pdTRUE)
            {
                u0_dbg_printf("Item wasn't received from queue after 500ms\n");
            }
            else
            {
                u0_dbg_printf("Rx pot data: %i\n", *rx_pot_addr_data);

                //Convert Pot Voltage Range between 0 and 3.3v to percentage
                //Pot Volt Percentage = ( ( ( ( pot_data mV / 1000 mV ) * 1V ) / 3.3V ) * 100 )
                //Compute Pot Volt % first! for duty cycle percentage for PWM1.1 - PWM1.3
                pot_volt_percent = ( ( *rx_pot_addr_data / 4095 ) * 100 );

                u0_dbg_printf("Pot Volt Percentage: %0.2f\n", pot_volt_percent);

                switch(RGB_COLOR)
                {
                    case RED:
                        pwm1_rgb.setDutyCycle(pwm1_rgb.PWM_PIN_2_0, pot_volt_percent);
                        pwm_red_dc = pwm1_rgb.getDutyCycle(pwm1_rgb.PWM_PIN_2_0); //PWM1.1 red
                        break;
                    case GREEN:
                        pwm1_rgb.setDutyCycle(pwm1_rgb.PWM_PIN_2_1, pot_volt_percent);
                         pwm_green_dc = pwm1_rgb.getDutyCycle(pwm1_rgb.PWM_PIN_2_1); //PWM1.2 green
                        break;
                    case BLUE:
                        pwm1_rgb.setDutyCycle(pwm1_rgb.PWM_PIN_2_2, pot_volt_percent);
                        pwm_blue_dc = pwm1_rgb.getDutyCycle(pwm1_rgb.PWM_PIN_2_2); //PWM1.3 blue
                        break;
                    case PURPLE:
                        pwm1_rgb.setDutyCycle(pwm1_rgb.PWM_PIN_2_0, pot_volt_percent);
                        pwm_red_dc = pwm1_rgb.getDutyCycle(pwm1_rgb.PWM_PIN_2_0); //PWM1.1 red

                        pwm1_rgb.setDutyCycle(pwm1_rgb.PWM_PIN_2_2, pot_volt_percent);
                        pwm_blue_dc = pwm1_rgb.getDutyCycle(pwm1_rgb.PWM_PIN_2_2); //PWM1.3 blue
                        break;
                    case BROWN:
                        pwm1_rgb.setDutyCycle(pwm1_rgb.PWM_PIN_2_0, pot_volt_percent);
                        pwm_red_dc = pwm1_rgb.getDutyCycle(pwm1_rgb.PWM_PIN_2_0); //PWM1.1 red

                        pwm1_rgb.setDutyCycle(pwm1_rgb.PWM_PIN_2_1, pot_volt_percent);
                        pwm_blue_dc = pwm1_rgb.getDutyCycle(pwm1_rgb.PWM_PIN_2_1); //PWM1.3 green
                        break;
                    case WHITE:
                        pwm1_rgb.setDutyCycle(pwm1_rgb.PWM_PIN_2_0, pot_volt_percent);
                        pwm_red_dc = pwm1_rgb.getDutyCycle(pwm1_rgb.PWM_PIN_2_0); //PWM1.1 red

                        pwm1_rgb.setDutyCycle(pwm1_rgb.PWM_PIN_2_1, pot_volt_percent);
                        pwm_green_dc = pwm1_rgb.getDutyCycle(pwm1_rgb.PWM_PIN_2_1); //PWM1.2 green

                        pwm1_rgb.setDutyCycle(pwm1_rgb.PWM_PIN_2_2, pot_volt_percent);
                        pwm_blue_dc = pwm1_rgb.getDutyCycle(pwm1_rgb.PWM_PIN_2_2); //PWM1.3 blue
                        break;
                }

                //Don't we want to read the PWM signal instead?
                u0_dbg_printf("PWM1.1 P2.0 Duty Cycle: %i\n", pwm_red_dc);
                u0_dbg_printf("PWN1.3 P2.1 Duty Cycle: %i\n", pwm_green_dc);
                u0_dbg_printf("PWN1.3 P2.2 Duty Cycle: %i\n", pwm_blue_dc);

            }

        }

        vTaskDelay(1000);
    }
}

void playRGBLEDApp(void)
{
    u0_dbg_printf("Entered playRGBLEDApp\n");

    //choose the color you want to set the RGB LED
    uint32_t RED = 0;
    uint32_t GREEN = 1;
    uint32_t BLUE = 2;
    uint32_t PURPLE = 3;
    uint32_t BROWN = 4;
    uint32_t WHITE = 5;

    uint32_t STACK_SIZE = 2048;
    uint32_t Priority_Low = 1;//Higher number = Higher Priority

    //Pass potentiometer voltage to RGB LED via semaphore
    xTaskCreate(vReadPotentiometer, "ReadPot", STACK_SIZE, NULL, Priority_Low, NULL);
    xTaskCreate(vDriveRGBLED, "DriverRGBLED", STACK_SIZE, (void *) RED, Priority_Low, NULL);
    vTaskStartScheduler();
}
