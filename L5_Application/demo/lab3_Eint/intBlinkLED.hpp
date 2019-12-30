/*
 * intBlinkLED.hpp
 *
 *  Created on: Feb 22, 2018
 *      Author: james
 */


#include <demo/lab3_Eint/LabGPIOInterrupts.hpp>
#include <demo/lab2_Gpio/LabGPIO.hpp>
#include "FreeRTOS.h"
#include "semphr.h"
#include "tasks.hpp"

#ifndef INTBLINKLED_HPP_
#define INTBLINKLED_HPP_


void c_eint3_handler(void);

// callback function for port 2 pin 1 interrupt
void p2_p1_func(void);

// waits on semaphore from callback function, then lights an LED
void vIntControlLED( void *pvParameters );

// creates vControlLED task, which only is triggered by an interrupt(s)
void vIntBlinkLED(void);

#endif /* INTBLINKLED_HPP_ */
