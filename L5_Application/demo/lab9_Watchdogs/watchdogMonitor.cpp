/*
 * watchdogMonitor.cpp
 *
 *  Created on: Apr 7, 2018
 *      Author: james
 */

#include <demo/lab9_Watchdogs/watchdogMonitor.hpp>
#include <stdio.h>
#include "time.h"
#include "stdlib.h"
#include <string>
#include <iostream>
#include "rtc_alarm.h"
#include "rtc.h"
#include "io.hpp"
#include "c_tlm_var.h"
#include "wireless.h"
#include "printf_lib.h"
#include "event_groups.h"
#include "soft_timer.hpp"

EventGroupHandle_t task_watchdog = xEventGroupCreate();
const uint32_t BIT_1 = (1 << 1); //0x02
const uint32_t BIT_2 = (1 << 2); //0x04
const uint32_t all_bits = (BIT_1 | BIT_2);

getLightData::getLightData() :
          scheduler_task("producer", 4*512, PRIORITY_MEDIUM)
{
    //run loop iterates every 1ms
    setRunDuration(1);
}

bool getLightData::init()
{
    //Create sensor_queue containing 1 32 bit value
    QueueHandle_t sensor_queue = xQueueCreate(1, sizeof(uint32_t));
    if(sensor_queue == 0)
    {
        u0_dbg_printf("Failed to create queue\n");
    }
    //producer shares its handle by name for communication, so consumer can later access it
    addSharedObject("sense_queue", sensor_queue);

    //check that the event group was created successfully
    if(task_watchdog == NULL)
    {
        u0_dbg_printf("failed to create event group due to insufficient heap space\n");
        return false;
    }
    return true;
}

bool getLightData::run(void *p)
{
    //Get Light Sample Percentage Readings
    uint8_t ls_data = LS.getPercentValue();
    light_sample = ls_data;
    //store light samples every 1ms until 100ms passes
    ls_samples[msCounter] = light_sample;
    //producer retrieves handle when running for tranmission to consumer
    QueueHandle_t sensor_queue = getSharedObject("sense_queue");
    //compute light sample average every 100ms and then reset msCounter
    if(msCounter == 99)
    {
        ls_sum = 0;
        u0_dbg_printf("Entered Producer\n");
        //Compute light sample sum across 100 elements
        for(uint32_t i = 0; i <= msCounter; ++i)
        {
            ls_sum = ls_sum + ls_samples[i];
            ls_samples[i] = 0;
        }
        //retrieve the number of samples
        num_of_ls = sizeof(ls_samples)/sizeof(ls_samples[0]);
        //compute light sample average
        ls_avg = (ls_sum)/(num_of_ls);
        //push item into queue, print error message if failed to do so
        if(!xQueueSend(sensor_queue, &ls_avg, portMAX_DELAY))
        {
            u0_dbg_printf("failed to send light sample\n");
        }
        //reset msCounter to start storing from 0 to 100 light samples again on next run iteration
        msCounter = 0;
    }

    //producer set bit 1 inside task_watchdog event group
    xEventGroupSetBits(task_watchdog, BIT_1);
    //increment counter index for array of light samples
    msCounter = msCounter + getRunDuration();
    return true;
}

pullLightData::pullLightData() :
          scheduler_task("consumer", 6*512, PRIORITY_MEDIUM)
{
    //run loop iterates every 400ms
    setRunDuration(400);
}

bool pullLightData::run(void *p)
{
    u0_dbg_printf("Entered Consumer.\n");
    //consumer retrieves queue handle that the producer shared
    QueueHandle_t sensor_queue = getSharedObject("sense_queue");
    //pull an item from the queue, but sleep if queue space isn't available
    if(xQueueReceive(sensor_queue, &ls_avg, portMAX_DELAY))
    {
        //create timestamp for sensor reading in M D Y H:M:S out of LPC_RTC data struct members
        const char current_date_time[] = "%u %u %u %u:%u:%u";
        char date_time_format[100];
            //stores current_date_time and its arguments into date_time_format char array
        sprintf(date_time_format, current_date_time, LPC_RTC->DOM, LPC_RTC->MONTH, LPC_RTC->YEAR, LPC_RTC->HOUR, LPC_RTC->MIN, LPC_RTC->SEC);

        u0_dbg_printf("Received Light Sensor AVG of 100 samples: %i\n", ls_avg);

        //set up sensor log format to write to sensor.txt file
        const char file_content[] = "%s, %i\n";

        //append 10 light samples to "sensor.txt", once msCounter reaches 10, then clear msCounter
        if(msCounter < 10)
        {
            //stores file_content into ls_avg_text and returns number of characters stored
            int num_of_chs = sprintf(ls_avg_text, file_content, date_time_format, ls_avg);
            u0_dbg_printf("file content written: %s\n", ls_avg_text);

            //append data to file 'sensor.txt'
            status = Storage::append("1:sensor.txt", ls_avg_text, num_of_chs, 0);
            if(status != FR_OK)
            {
                u0_dbg_printf("Write failed to 'sensor.txt' due to: %i.\n", status);
            }
            u0_dbg_printf("SD compatibility: %i\n", status);
            Storage::read("1:sensor.txt", read_buffer, sizeof(read_buffer)-1, offset);
            u0_dbg_printf("Data in sensor.txt:\t %s\n", read_buffer);
            msCounter++;
            offset++;
        }
        else
        {
            //reset counter after writing to file 10 times
            msCounter = 0;
        }
    }
    else
    {
        u0_dbg_printf("Failed to receive item from queue\n");
    }

    //consumer set bit 2 inside task_watchdog event group
    xEventGroupSetBits(task_watchdog, BIT_2);

    //set bit 2 using FreeRTOS event group
    return true;
}

watchdogMonitor::watchdogMonitor() :
          scheduler_task("watchdog", 4*512, PRIORITY_HIGH)
{
    setRunDuration(400);
}

//monitors operation of producer and consumer
bool watchdogMonitor::run(void *p)
{
    u0_dbg_printf("Watchdog monitoring both tasks.\n");

    //LOG message
    const char file_content[] = "WARN: %s, %s\n";
    const char cpu_info[] = "%10s %s %2u %5u %4u %10u us\n";
    const char cpu_overhead_info[] = "%10s --- -- ----- %4u %10u uS\n";
    uint32_t uxBits;
    const TickType_t xTicksToWait = 1000 / portTICK_PERIOD_MS;

    //Wait max of 1000ms for both bit_1 and bit_2 to be set
    uxBits = xEventGroupWaitBits(
            task_watchdog,  //Event group being tested
            all_bits,       //test bits 1 and 2
            pdTRUE,         //clear BIT_1 & BIT_2 before returning
            pdTRUE,         //Wait for all bits to be set from uxBitsToWaitFor
            xTicksToWait ); //Wait a max 1000ms for either bit to be set

    //if BOTH Bits are set, print both tasks are operating fine
    if( ( uxBits & ( BIT_1 | BIT_2 ) ) == ( BIT_1 | BIT_2 ) )
    {
        /* xEventGroupWaitBits() returned because both bits were set. */
        u0_dbg_printf("Producer and Consumer Are Responding!\n");
    }
    //else if BIT_1 doesn't equal 0, then consumer isn't responding
    else if( ( uxBits & BIT_1 ) != 0 )
    {
        /* xEventGroupWaitBits() returned because just BIT_1 was set. */
        u0_dbg_printf("Consumer didn't set BIT_2: %i in watchdog EG.\n", uxBits);
        //Append information that Consumer task isn't responding into "stuck.txt"
            //Get timestamp of when Consumer didn't respond
        const char current_date_time[] = "%u %u %u %u:%u:%u";
        char date_time_format[100];
        sprintf(date_time_format, current_date_time, LPC_RTC->DOM, LPC_RTC->MONTH, LPC_RTC->YEAR, LPC_RTC->HOUR, LPC_RTC->MIN, LPC_RTC->SEC);

        const char message[] = "Consumer not responding...\n";
        //stores file_content into ls_avg_text and returns num of characters stored
        int num_of_chs = sprintf(log_text, file_content, date_time_format, message);

        //append log to file 'stuck.txt'
        status = Storage::append("1:stuck.txt", log_text, num_of_chs, 0);
        //verify append to file succeeded
        if(status != FR_OK)
        {
            u0_dbg_printf("Write failed to 'stuck.txt' due to: %i.\n", status);
        }
        else if(status == FR_OK)
        {
            u0_dbg_printf("SD compatibility: SUCCEDED\n");
        }
        Storage::read("1:stuck.txt", read_log, sizeof(read_log)-1, stuck_offset);
        u0_dbg_printf("Data in stuck.txt:\t %s\n", read_log);
        stuck_offset++;
    }
    //else if BIT_2 doesn't equal 0, then producer isn't responding
    else if( ( uxBits & BIT_2 ) != 0 )
    {
        /* xEventGroupWaitBits() returned because just BIT_2 was set. */
        u0_dbg_printf("Producer didn't set BIT_1: %i in watchdog EG.\n", uxBits);
        //Append information that Consumer task isn't responding into "stuck.txt"
            //Get timestamp of when Producer didn't respond
        const char current_date_time[] = "%u %u %u %u:%u:%u";
        char date_time_format[100];
        sprintf(date_time_format, current_date_time, LPC_RTC->DOM, LPC_RTC->MONTH, LPC_RTC->YEAR, LPC_RTC->HOUR, LPC_RTC->MIN, LPC_RTC->SEC);

        const char message[] = "Producer not responding...\n";
        //stores file_content into ls_avg_text and returns num of characters stored
        int num_of_chs = sprintf(log_text, file_content, date_time_format, message);

        //append data to file 'stuck.txt'
        status = Storage::append("1:stuck.txt", log_text, num_of_chs, 0);
        if(status != FR_OK)
        {
            u0_dbg_printf("Write failed to 'stuck.txt' due to: %i.\n", status);
        }
        u0_dbg_printf("SD compatibility: %i\n", status);
        Storage::read("1:stuck.txt", read_log, sizeof(read_log)-1, stuck_offset);
        u0_dbg_printf("Data in stuck.txt:\t %s\n", read_log);
        stuck_offset++;
    }
    //BIT_1 and BIT_2 were not set, so problem occurring with both tasks
    else
    {
        /* xEventGroupWaitBits() returned since xTicksToWait ticks passed
          neither BIT_1 nor BIT_2 were set. */
        u0_dbg_printf("BIT_1 and BIT_2 weren't set in watchdog EG.\n");
        //Append information that BOTH TASKS aren't responding into "stuck.txt"
            //Get timestamp to store with message
        const char current_date_time[] = "%u %u %u %u:%u:%u";
        char date_time_format[100];
        sprintf(date_time_format, current_date_time, LPC_RTC->DOM, LPC_RTC->MONTH, LPC_RTC->YEAR, LPC_RTC->HOUR, LPC_RTC->MIN, LPC_RTC->SEC);

        const char message[] = "Producer and Consumer not responding...\n";
        //stores file_content into ls_avg_text and returns num of characters stored
        int num_of_chs = sprintf(log_text, file_content, date_time_format, message);

        //append data to file 'stuck.txt'
        status = Storage::append("1:stuck.txt", log_text, num_of_chs, 0);
        if(status != FR_OK)
        {
            u0_dbg_printf("Write failed to 'stuck.txt' due to: %i.\n", status);
        }
        u0_dbg_printf("SD compatibility: %i\n", status);
        Storage::read("1:stuck.txt", read_log, sizeof(read_log)-1, stuck_offset);
        u0_dbg_printf("Data in stuck.txt:\t %s\n", read_log);
        stuck_offset++;
    }
    //Extra Credit: save CPU usage info to 'cpu.txt' every 60 seconds
    if(secCounter == 59)
    {
        //Referenced the CPU Usage Info code from 'TaskListHandler'
        // Enum to char : eRunning, eReady, eBlocked, eSuspended, eDeleted
        const char * const taskStatusTbl[] = { "RUN", "RDY", "BLK", "SUS", "DEL" };

        // Limit the tasks to avoid heap allocation.
        const unsigned portBASE_TYPE maxTasks = 16;
        TaskStatus_t tStatus[maxTasks];
        uint32_t totalRunTime = 0;
        uint32_t tasksRunTime = 0;
        const unsigned portBASE_TYPE uxArraySize =
                uxTaskGetSystemState(&tStatus[0], maxTasks, &totalRunTime);

        u0_dbg_printf("%10s Sta Pr Stack CPU%%          Time\n", "Name");
        for(unsigned priorityNum = 0; priorityNum < configMAX_PRIORITIES; priorityNum++)
        {
            /* Print in sorted priority order */
            for (unsigned i = 0; i < uxArraySize; i++) {
                TaskStatus_t *e = &tStatus[i];
                if (e->uxBasePriority == priorityNum) {
                    tasksRunTime += e->ulRunTimeCounter;

                    const uint32_t cpuPercent = (0 == totalRunTime) ? 0 : e->ulRunTimeCounter / (totalRunTime/100);
                    const uint32_t timeUs = e->ulRunTimeCounter;
                    const uint32_t stackInBytes = (4 * e->usStackHighWaterMark);

                    //stores cpu_info into mCpu_log and returns num of characters stored
                    int num_of_chs = sprintf(mCpu_log, cpu_info, e->pcTaskName, taskStatusTbl[e->eCurrentState], e->uxBasePriority, stackInBytes, cpuPercent, timeUs);

                    //append data to 'cpu.txt' with the number of characters written to file
                    status = Storage::append("1:cpu.txt", mCpu_log, num_of_chs, 0);
                    if(status != FR_OK)
                    {
                        u0_dbg_printf("Write failed to 'cpu.txt' due to: %i.\n", status);
                    }
                    u0_dbg_printf("SD compatibility: %i\n", status);
                    Storage::read("1:cpu.txt", read_log, sizeof(read_log)-1, cpu_offset);
                    u0_dbg_printf("Data in cpu.txt:\t %s\n", read_log);
                    cpu_offset++;
                }
            }
        }

        // Overhead is the time not accounted towards any of the tasks.
        // For example, when an ISR happens, that is not part of a task's CPU usage.
        const uint32_t overheadUs = (totalRunTime - tasksRunTime);
        const uint32_t overheadPercent = overheadUs / (totalRunTime / 100);

        //stores cpu_info into mCpu_log and returns num of characters stored
        int num_of_chs = sprintf(mCpu_log, cpu_overhead_info, "(overhead)", overheadPercent, overheadUs);

        //append data to file 'cpu.txt'
        status = Storage::append("1:cpu.txt", mCpu_log, num_of_chs, 0);
        if(status != FR_OK)
        {
            u0_dbg_printf("Write failed to 'cpu.txt' due to: %i.\n", status);
        }
        u0_dbg_printf("SD compatibility: %i\n", status);
        Storage::read("1:cpu.txt", read_log, sizeof(read_log)-1, cpu_offset);
        u0_dbg_printf("Data in cpu.txt:\t %s\n", read_log);
        cpu_offset++;
        //Referenced the CPU Usage Info code from 'TaskListHandler'
        secCounter = 0;
    }

    u0_dbg_printf("secCounter = %i\n", secCounter);
    //increment the seconds counter after every loop
    secCounter++;
    return true;
}

void runWatchdogMonitor()
{
    //we utilize terminal task to execute our suspend and resume producer(getLightData) and consumer(pullLightData) tasks
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    //able to send light sensor data from producer to consumer (success)
    scheduler_add_task(new getLightData());
    scheduler_add_task(new pullLightData());
    scheduler_add_task(new watchdogMonitor());
    scheduler_start(); ///< This shouldn't return
}
