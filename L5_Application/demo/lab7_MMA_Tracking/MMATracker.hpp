/*
 * MMATracker.hpp
 *
 *  Created on: Mar 21, 2018
 *      Author: james
 */

#ifndef MMATRACKER_HPP_
#define MMATRACKER_HPP_

#include "FreeRTOS.h"
#include "stdint.h"
#include "queue.h"
#include "printf_lib.h"
#include "task.h"
#include "LPC17xx.h"
#include "io.hpp"

/**
 * Captures triple-axis x, y, z coordinates from accelerometer
 * Determines if orientation is UP, DOWN, LEFT or RIGHT
 * based on a threshold const value along the appropriate axis
 */
void producer(void *p); /* LOW Priority */

void consumer(void *p); /* HIGH Priority */

void runMMATracker();

#endif /* MMATRACKER_HPP_ */
