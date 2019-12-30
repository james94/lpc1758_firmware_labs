/*
 * multi_tasks.cpp
 *
 *  Created on: Jan 28, 2018
 *      Author: james
 *      Lab Assignment 1: Multiple Tasks
 */

#include "multi_tasks.hpp"

void vTaskOneCode(void *pvParam)
{
    while(1)
    {
        uart0_puts("aaaaaaaaaaaaaaaaaaaa");
        vTaskDelay(100);
    }
}

void vTaskTwoCode(void *pvParam)
{
    while(1)
    {
        uart0_puts("bbbbbbbbbbbbbbbbbbbb");
        vTaskDelay(100);
    }
}

void vTaskPrint(void)
{
    const uint32_t STACK_SIZE = 128;

//    //Priority Scenario 1: TaskA = 1, TaskB = 1
//    xTaskCreate(vTaskOneCode, "TaskA", STACK_SIZE, 0, 2, 0);
//    //Error message when void pointer param not included:
//    //invalid conversion from 'void (*)()' to 'TaskFunction_t {aka void (*)(void*)}' [-fpermissive]
//    xTaskCreate(vTaskTwoCode, "TaskB", STACK_SIZE, 0, 1, 0);
//
//    //Priority Scenario 2: TaskA = 2, TaskB = 1
//    xTaskCreate(vTaskOneCode, "TaskA", STACK_SIZE, 0, 2, 0);
//    //Error message when void pointer param not included:
//    //invalid conversion from 'void (*)()' to 'TaskFunction_t {aka void (*)(void*)}' [-fpermissive]
//    xTaskCreate(vTaskTwoCode, "TaskB", STACK_SIZE, 0, 1, 0);

    //Priority Scenario 3: TaskA = 1, TaskB = 2
    xTaskCreate(vTaskOneCode, "TaskA", STACK_SIZE, 0, 1, 0);
    //Error message when void pointer param not included:
    //invalid conversion from 'void (*)()' to 'TaskFunction_t {aka void (*)(void*)}' [-fpermissive]
    xTaskCreate(vTaskTwoCode, "TaskB", STACK_SIZE, 0, 2, 0);
    vTaskStartScheduler();
}



