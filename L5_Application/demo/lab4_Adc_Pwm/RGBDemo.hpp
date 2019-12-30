/*
 * RGBDemo.hpp
 *
 *  Created on: Feb 25, 2018
 *      Author: james
 */

#ifndef RGBDEMO_HPP_
#define RGBDEMO_HPP_

#include <demo/lab2_Gpio/LabGPIO.hpp>
#include <demo/lab4_Adc_Pwm/ADCDriver.hpp>
#include <demo/lab4_Adc_Pwm/PWMDriver.hpp>
#include <cstddef>
#include "tasks.hpp"

/*Test App code to verify ADC + PWM Drivers worked*/
/*Verification Completed: 2/28/18 6:32PM */
void testAnalogLight(void *params);
void readPot(void *params);
void printAnalogLight(void);
void printPotVoltage(void);
void vTestPWMChBlinkLED(void *params);
void vTestPWMChDimLED(void *params);
void printDutyCycle(void);

/*MVP Application: Interface Potentiometer with an RGB LED*/
void vReadPotentiometer(void *params);
void vDriveRGBLED(void *params);
void playRGBLEDApp(void);

#endif /* RGBDEMO_HPP_ */
