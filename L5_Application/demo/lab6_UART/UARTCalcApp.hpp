/*
 * UARTCalcApp.hpp
 *
 *  Created on: Mar 15, 2018
 *      Author: james
 */

#ifndef UARTCALCAPP_HPP_
#define UARTCALCAPP_HPP_

#include <demo/lab6_UART/UARTDriver.hpp>
#include <cstddef>
#include "tasks.hpp"
#include <cstdbool>

/*Utility functions to help the tasks*/

//print elements of char array
void printCharArr(const char *result, const uint32_t MAX_ITEMS);

//Prints char array integer result without leading zeros
void printCharArrResult(const char *c_result, const char operand1, const char operand2, const char operation, const uint32_t MAX_ITEMS);

//pointer to objects of type char
void wIntToCharArr(int32_t i_result, char *c_result, uint32_t arr_size);

/*Tasks for UART Calculator Application*/

//vUser:
//enters input for the calculator: 2 numbers, 1 operator
void vUser(void *params);

//vALU:
//receives user's math computation request, then performs the computation.
//PENDING: send the result back to the User
void vALU(void *params);

/**
 * Loopback test:
 *
 * Tests both tasks (vUser and vALU) on one board to verify
 * vUser receives their requested mathematical operation result
 * of two operands
 */
void runUARTDriverTest(void);

/**
 * MVP Demo Application:
 *
 * runUser() is for Device 1, which sends 2 operands and operator
 * to Device 2, which executes runALU() to do the computation,
 * then Device 2 sends the result back to Device 1 and runUser()
 * prints the result to the console(hercules)
 */

void runUser(void);

void runALU(void);

/**
 * Extra Credit Application:
 *
 * Interrupt Driven Rx
 * GPIO Buttons and 7-Segment HID
 */

void c_uart3_handler(void);

void c_callbackIntReceive(void);

/**
 * The two tasks utilize queues to get data from the UARTn Interrupt Rx
 */
//void vIntUser(void *params);
//void vIntALU(void *params);
void vTestIntRx(void *params);

#endif /* UARTCALCAPP_HPP_ */
