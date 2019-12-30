/*
 * LabGPIOInterrupts.hpp
 *
 *  Created on: Feb 19, 2018
 *      Author: james
 */

#ifndef LABGPIOINTERRUPTS_HPP_
#define LABGPIOINTERRUPTS_HPP_

#include "LPC17xx.h"
#include "stdbool.h"
#include "printf_lib.h"
#include <cstddef>

typedef enum
{
    RISING,
    FALLING,
    BOTH
} InterruptCondition_E;

class LabGPIOInterrupts
{
protected:
    /**
     * Allocate a lookup table matrix here of function pointers (avoid dynamic allocation)
     * Upon attachInterruptHandler(), you will store the user's function callback
     * Upon the EINT3 interrupt, you will find out which callback to invoke based on Port/Pin status
     * Be clever here. How can you do this such that you and the cpu do the least amount of work.
     */


    //void function pointers matrix: row 0 = rising, row 1 = falling
//    void (*GPIOInt_Port0_R_F[2][32])(void)
//    {
//        //Port 0 Pins [0-31] Rising
//        //Port 0 Pins [0-31] Falling
//    };
//    void (*GPIOInt_Port2_R_F[2][32])(void);

    //rising edge interrupts on port 0 pins [0-31]
    void (*GPIOInt_Port0_Rising[32])(void);
    void (*GPIOInt_Port0_Falling[32])(void);

    //edge interrupts on port 2 pins [0-31]
    void (*GPIOInt_Port2_Rising[32])(void);
    void (*GPIOInt_Port2_Falling[32])(void);

public:

    /**
     * Optional: LabGPIOInterrupts is a singleton, meaning, only one instance can exist at a time.
     * Look up how to implement this. It is best to not allocate memory in the constructor and leave complex
     * code to the init() that you call in your main()
     */
    LabGPIOInterrupts();

    /**
     * Configure NVIC to notice EINT3 IRQs: use NVIC_EnableIRQ()
     */
    void init();

    /**
     * This handler will place a function pointer within the lookup table for the
     * externalIRQHandler to find.
     *
     * @param[in] port          specify the GPIO port
     * @param[in] pin           specify the GPIO pin to assign an ISR to
     * @param[in] pin_isr       function to run when interrupt event occurs
     * @param[in] condition     condition for the interrupt to occur on: RISING, FALLING, BOTH edges
     * @return returns if valid ports, pins, isrs were supplied and pin isr insertion was successful
     */
    bool attachInterruptHandler(uint8_t port, uint8_t pin, void (*pin_isr)(void), InterruptCondition_E condition);

    /**
     * This function is invoked by the CPU (through c_eint3_handler) asynchronously when a Port/Pin
     * interrupt occurs. This function is where you will check the Port status, such as IO0IntStatF,
     * and then invoke the user's registered callback and find the entry in your lookup table.
     *
     * VERY IMPORTANT!
     *  - Be sure to clear the interrupt flag that caused this interrupt, or this function will be called
     *    repetitively and lock your system.
     *  - NOTE that your code needs to be able to handle two GPIO interrupts occurring at the same time.
     */
    void handle_interrupt(void);

    virtual ~LabGPIOInterrupts();
};

#endif /* LABGPIOINTERRUPTS_HPP_ */
