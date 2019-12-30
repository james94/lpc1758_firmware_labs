/*
 * Test.h
 *
 *  Created on: May 13, 2018
 *      Author: james
 */

#ifndef TEST_H_
#define TEST_H_

#include "tasks.hpp"
#include "printf_lib.h"

extern QueueHandle_t mp3_data_queue;

void sendAudioData(void *vParam);
void receiveAudioData(void *vParam);

void runSDCardToDecoder();

#endif /* TEST_H_ */
