/*
 * multi_tasks.hpp
 *
 *  Created on: Jan 28, 2018
 *      Author: james
 */

#ifndef MULTI_TASKS_HPP_
#define MULTI_TASKS_HPP_

#include "tasks.hpp"
#include "FreeRTOS.h"
#include "uart0_min.h"

//Prints 20 characters of 'a', then sleeps for 100ms
void vTaskOneCode(void *pvParam);

//Prints 20 characters of 'b', then sleeps for 100ms
void vTaskTwoCode(void *pvParam);

//Creates two tasks with STACK_SIZE=128
void vTaskPrint(void);

#endif /* MULTI_TASKS_HPP_ */
