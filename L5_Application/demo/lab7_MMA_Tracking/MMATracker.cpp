/*
 * MMATracker.cpp
 *
 *  Created on: Mar 21, 2018
 *      Author: james
 */

#include <demo/lab7_MMA_Tracking/MMATracker.hpp>

QueueHandle_t mma_queue;

/**
 * Captures triple-axis x, y, z coordinates from accelerometer
 * Determines if orientation is UP, DOWN, LEFT or RIGHT
 * based on a threshold const value along the appropriate axis
 */
void producer(void *p) /* LOW Priority */
{
    //data structure mma to hold the x, y, z axis data
    struct mma
    {
        int32_t x = 0, y = 0, z = 0;
    } mma_data;

    enum orientation
    {
        UP = 0,
        DOWN = 1,
        LEFT = 2,
        RIGHT = 3,
        INVALID = 4
    } mma_orient;

    //Board Linearly stationed when
    //STATIONED (INVALID): x = 0, y = 0, z = 1000;

    //x-axis threshold for tilting RIGHT or LEFT
    const int32_t RIGHT_TILT = -500;
    const int32_t LEFT_TILT = 500;
    //y-axis threshold for tilting UP or DOWN
    const int32_t UP_TILT = -500;
    const int32_t DOWN_TILT = 500;

    //z-axis threshold for stationed (INVALID) is greater 980
    const int32_t STATIONED = 950;

    while(1)
    {
        //get the x, y, z axis accelerometer data
        mma_data.x = AS.getX();
        mma_data.y = AS.getY();
        mma_data.z = AS.getZ();

        //Compute whether the accelerometer is rotated up, down, left, right
        //UP: x = -53, y = 761, z = 677;
        //if greater than 500 on z-axis looking up
        if(mma_data.x < RIGHT_TILT)
        {
            mma_orient = RIGHT;
        }
        //else if less than -500 on z-axis looking down
        else if(mma_data.x > LEFT_TILT)
        {
            mma_orient = LEFT;
        }
        else if(mma_data.y < UP_TILT)
        {
            mma_orient = UP;
        }
        else if(mma_data.y > DOWN_TILT)
        {
            mma_orient = DOWN;
        }
        else if(mma_data.z > STATIONED)
        {
            mma_orient = INVALID;
        }


        // This xQueueSend() will internally switch context over to the "consumer" task
        // because it is higher priority than this "producer" task

        // Then, when the consumer task sleeps, we will resume out of xQueueSend()
        // and go over to the next line
        u0_dbg_printf("Producer about to send messages into the queue\n\n");
        if(xQueueSend(mma_queue, &mma_orient, 0) != pdTRUE)
        {
            u0_dbg_printf("Failed to send data into mma_queue\n");
        }
        else
        {
            //Right when inside xQueueSend, cooperative context switch to Consumer,
            //then when done come back and run code starting on the following line
            u0_dbg_printf("Producer Sending MMA Triple Axis Data to Consumer\n");
            u0_dbg_printf("Prod: Sent x-axis %i into the mma_queue\n", mma_data.x);
            u0_dbg_printf("Prod: Sent y-axis %i into the mma_queue\n", mma_data.y);
            u0_dbg_printf("Prod: Sent z-axis %i into the mma_queue\n\n", mma_data.z);

        }
        u0_dbg_printf("Producer sent data to the queue\n\n");
        // iterate every 1 sec
        vTaskDelay(1000);
    }

}

/**
 * Receives the enum orientation from the Producer task
 * and prints out the board orientation to the console
 * as UP, DOWN, LEFT, RIGHT or INVALID
 */
void consumer(void *p) /* HIGH Priority */
{

    enum orientation
    {
        UP = 0,
        DOWN = 1,
        LEFT = 2,
        RIGHT = 3,
        INVALID = 4
    } mma_orient;

    while(1)
    {
        // Waits on orientation enumeration to be sent by producer task
        if(xQueueReceive(mma_queue, &mma_orient, portMAX_DELAY) != pdTRUE)
        {
            u0_dbg_printf("Failed to receive data into mma_queue\n");
        }
        else
        {
            u0_dbg_printf("Consumer: Receiving data from Producer\n\n");
            //Print the appropriate orientation of the board
            switch(mma_orient)
            {
                case UP:
                    u0_dbg_printf("UP\n\n");
                    break;
                case DOWN:
                    u0_dbg_printf("DOWN\n\n");
                    break;
                case LEFT:
                    u0_dbg_printf("LEFT\n\n");
                    break;
                case RIGHT:
                    u0_dbg_printf("RIGHT\n\n");
                    break;
                case INVALID:
                    u0_dbg_printf("INVALID\n\n");
                    break;
            }
        }
        u0_dbg_printf("Consumer done printing received orientation data\n");
        vTaskDelay(1000);
    }
}

void runMMATracker()
{
    // Create Queue
    // FreeRTOS Queue API will perform Cooperative Context Switches when
    // a higher priority task is waiting to receive data in the queue
    // from a lower priority task
    mma_queue = xQueueCreate(1, sizeof(int32_t));

    // Set priority level
    const uint32_t Priority_Low = 1;
    const uint32_t Priority_High = 2;

    // Create both tasks
    xTaskCreate(producer, "producer", 1024, NULL, Priority_Low, NULL);
    xTaskCreate(consumer, "consumer", 1024, NULL, Priority_High, NULL);

    // Start FreeRTOS Scheduler, which runs each task
    // Will perform preemptive context switches if both tasks have same priority level
    vTaskStartScheduler();
}
