/*
 * LabGPIOInterrupts.cpp
 *
 *  Created on: Feb 19, 2018
 *      Author: james
 */

#include <demo/lab3_Eint/LabGPIOInterrupts.hpp>

// Public Members

// Initialize GPIO Port 0 and Port 2 Int Matrix
LabGPIOInterrupts::LabGPIOInterrupts()
{
    for(int i = 0; i < 32; ++i)
    {
        GPIOInt_Port0_Rising[i] = NULL;
        GPIOInt_Port0_Falling[i] = NULL;
        GPIOInt_Port2_Rising[i] = NULL;
        GPIOInt_Port2_Falling[i] = NULL;
    }
}

//GPIO Interrupt Driver Configuration
void LabGPIOInterrupts::init()
{
    // Configure NVIC to notice EINT3 IRQs
    NVIC_EnableIRQ(EINT3_IRQn);
}

/**
 * This handler will place a function pointer within the lookup table for the
 * externalIRQHandler to find.
 */
bool LabGPIOInterrupts::attachInterruptHandler(uint8_t port, uint8_t pin, void (*pin_isr)(void), InterruptCondition_E condition)
{
    bool int_status = false;
    //handles one dimensional array
    // port and pin, which will pass to the look up table
    if(port == 0)
    {
        if(condition == RISING)
        {
            LPC_GPIOINT->IO0IntEnR |= (1 << pin);
            GPIOInt_Port0_Rising[pin] = pin_isr;
            int_status = true;
            u0_dbg_printf("Attached RE Int P%i P%i", port, pin);
        }
        else if(condition == FALLING)
        {
            LPC_GPIOINT->IO0IntEnF |= (1 << pin);
            GPIOInt_Port0_Falling[pin] = pin_isr;
            int_status = true;
            u0_dbg_printf("Attached FE Int P%i P%i", port, pin);
        }
        else if(condition == BOTH)
        {
            LPC_GPIOINT->IO0IntEnR |= (1 << pin);
            LPC_GPIOINT->IO0IntEnF |= (1 << pin);
            GPIOInt_Port0_Rising[pin] = pin_isr;
            GPIOInt_Port0_Falling[pin] = pin_isr;
            u0_dbg_printf("Attached BE Int P%i P%i", port, pin);
            int_status = true;
        }
        else
        {
            u0_dbg_printf("Port 0 Invalid Condition\n");
            int_status = false;
        }
    }
    else if(port == 2)
    {
        switch(condition)
        {
            case RISING:
                LPC_GPIOINT->IO2IntEnR |= (1 << pin);
                GPIOInt_Port2_Rising[pin] = pin_isr;
                u0_dbg_printf("Attached RE Int P%i P%i", port, pin);
                int_status = true;
                break;
            case FALLING:
                LPC_GPIOINT->IO2IntEnF |= (1 << pin);
                GPIOInt_Port2_Falling[pin] = pin_isr;
                u0_dbg_printf("Attached RE Int P%i P%i", port, pin);
                int_status = true;
                break;
            case BOTH:
                LPC_GPIOINT->IO2IntEnR |= (1 << pin);
                LPC_GPIOINT->IO2IntEnF |= (1 << pin);
                GPIOInt_Port2_Rising[pin] = pin_isr;
                GPIOInt_Port2_Falling[pin] = pin_isr;
                u0_dbg_printf("Attached RE Int P%i P%i", port, pin);
                int_status = true;
                break;
            default:
                u0_dbg_printf("Port 2 Invalid Condition\n");
                int_status = false;
                break;
        }
    }
    else
    {
        u0_dbg_printf("Invalid Port %i\n", port);
        int_status = false;
    }

    return int_status;
}

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
void LabGPIOInterrupts::handle_interrupt(void)
{
//    u0_dbg_printf("Entered Handler\n");
    for(int i = 0; i < 32; ++i)
    {
        //check if interrupt has been detected on RISING EDGE PORT0 pin[0-31]
        if(LPC_GPIOINT->IO0IntStatR & (1 << i))
        {
            u0_dbg_printf("Int Stat RE Occurred\n");
            u0_dbg_printf("P0 P%i\n", i);
            if(GPIOInt_Port0_Rising[i] != NULL)
            {
                GPIOInt_Port0_Rising[i]();
                u0_dbg_printf("Called Callback\n");
            }
        }//check interrupt detected on FALLING EDGE PORT0 pin[0-31]
        if(LPC_GPIOINT->IO0IntStatF & (1 << i))
        {
            u0_dbg_printf("Int Stat FE Occurred\n");
            u0_dbg_printf("P0 P%i\n", i);
            if(GPIOInt_Port0_Falling[i] != NULL)
            {
                GPIOInt_Port0_Falling[i]();
                u0_dbg_printf("Called Callback\n");
            }
        }//check interrupt detected on RISING EDGE PORT2
        if(LPC_GPIOINT->IO2IntStatR & (1 << i))
        {
            u0_dbg_printf("Int Status RE Occurred\n");
            u0_dbg_printf("P2 P%i\n", i);
            if(GPIOInt_Port2_Rising[i] != NULL)
            {
                GPIOInt_Port2_Rising[i]();
                u0_dbg_printf("Called Callback\n");
            }
        }//check interrupt detected on FALLING EDGE PORT2
        if(LPC_GPIOINT->IO2IntStatF & (1 << i))
        {
            u0_dbg_printf("Int Status Falling Occurred\n");
            u0_dbg_printf("P2 P%i\n", i);
            if(GPIOInt_Port2_Falling[i] != NULL)
            {
                GPIOInt_Port2_Falling[i]();
                u0_dbg_printf("Called Callback\n");
            }
        }
    }
    LPC_GPIOINT->IO0IntClr = 0xFFFFFFFF;
    LPC_GPIOINT->IO2IntClr = 0xFFFFFFFF;
}


LabGPIOInterrupts::~LabGPIOInterrupts()
{
    // TODO Auto-generated destructor stub
}
