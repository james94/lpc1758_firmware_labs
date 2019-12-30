/*
 * blinkLED.hpp
 *
 *  Created on: Jan 30, 2018
 *      Author: james
 */

#ifndef BLINKLED_HPP_
#define BLINKLED_HPP_

#include "LabGPIO.hpp"
#include "FreeRTOS.h"
#include "tasks.hpp"
#include "stdbool.h"
#include "printf_lib.h"

//toggles LED when flag set
extern bool toggle_int, toggle_ext, toggle_easter;
//bool toggle_ext = false;

//Reads from internal OR external switch, depends on input parameter,
//connected to switches
//If button RELEASED, then sets global variable
void vReadSwitch( void *pvParameters );

//Toggles state of internal OR external LED, depends on input parameter
//and clears global variable
void vControlLED( void *pvParameters );

//Creates four tasks with STACK_SIZE=256 each
//blinks internal or external LEDs when internal or external SWs RELEASED
void vBlinkLED(void);

#endif /* BLINKLED_HPP_ */
