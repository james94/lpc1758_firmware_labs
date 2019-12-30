/*
 * watchdogMonitor.hpp
 *
 *  Created on: Apr 7, 2018
 *      Author: james
 */

#ifndef WATCHDOGMONITOR_HPP_
#define WATCHDOGMONITOR_HPP_

#include "scheduler_task.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "storage.hpp"
#include "tasks.hpp"
#include "periodic_scheduler/periodic_callback.h"
#include "uart2.hpp"
#include "uart3.hpp"
#include "utilities.h"

//writes to sensor_queue every 1ms
class getLightData : public scheduler_task
{
public:
    getLightData();
    bool init();
    bool run(void *p);
private:
    uint32_t ls_avg = 0;
    uint32_t msCounter = 0;
    uint8_t light_sample = 0;
    uint16_t num_of_ls = 0;
    int ls_sum = 0;
    uint8_t ls_samples[100] = {0};
};

//pulls data off sensor_queue every 100ms
class pullLightData : public scheduler_task
{
public:
    pullLightData();
    bool run(void *p);
private:
    uint32_t ls_avg = 0;
    char ls_avg_text[1000];
    uint32_t msCounter = 0;
    char read_buffer[100];
    uint32_t offset = 0;
    FRESULT status;
};

//software watchdog monitors status of producer & consumer & cpu usage info
class watchdogMonitor : public scheduler_task
{
public:
    watchdogMonitor();
    bool run(void *p);
private:
    char log_text[512];
    char mCpu_log[512];
    char read_log[100];
    FRESULT status;
    uint32_t secCounter = 0;
    uint32_t stuck_offset = 0;
    uint32_t cpu_offset = 0;
};

//creates three tasks and adds them to RTOS scheduler
void runWatchdogMonitor(void);

#endif /* WATCHDOGMONITOR_HPP_ */
