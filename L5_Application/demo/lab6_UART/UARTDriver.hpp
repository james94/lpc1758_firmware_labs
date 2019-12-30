/*
 * UARTDriver.hpp
 *
 *  Created on: Mar 13, 2018
 *      Author: james
 */

#ifndef UARTDRIVER_HPP_
#define UARTDRIVER_HPP_

#include "LPC17xx.h"
#include "FreeRTOS.h"
#include "printf_lib.h"
#include "queue.h"

/**
 * Requirements:
 * Design UART Driver to support UART-2 and UART-3 (Done and Tested)

 * ALU app must support operators (Done and Tested):
 *      + Addition
 *      - Subtraction
 *      * Multiplication
 */

/**
 * Extra Credit:
 *
 * UART Interrupt Implementation:
 * UART receive should be interrupt driven (In-Progress)
 *      When data arrives, stored into FreeRTOS queue inside UART Rx Interrupt
 *      UART receive function should dequeue the data (Function done and Tested)
 *
 * GPIO Buttons and 7-Seg HID:
 * On-Board Buttons and 7-segment dislay as human interface device (HID) (Not Started)
 */

/**
 * What to turn in:
 *
 * Submit all relevant files (includes previous lab code if used)
 * Turn in any screenshots of terminal output
 * Logic Analyzer Screenshots
 *      - Waveform of device 1 UART Tx sending a digit and operator to device 2
 *          - These can be in separate images if you can't fit everything in one image
 *      - Waveform of device 2 UART Tx sending result back to device 1
 *      - Whole window screenshot with "Decoded Protocols" (lower right hand side of window) clearly legible
 */

typedef enum
{
    RDAInterrupt = 2,
    THREInterrupt = 1,
    RXLSInterrupt = 3,
    CTIInterrupt = 6,
} UARTInterruptCondition_E;

class UARTDriver {
private:
    /* Pointer array of pointer objects: LPC_UART2 and LPC_UART3 that can be used
     * to point to memory locations of UART2 and UART3 peripherals*/
    static constexpr LPC_UART_TypeDef * UART[] = { LPC_UART2, LPC_UART3 };

    //RBR Interrupt: Receive Data Available for UARTn. Also controls Character Receive Time-out interrupt
    //void (*UART_2_Int_Port2_P9_RDA)(void);

    // UART RDA interrupt function enabled for UART-2 or UART-3
    void (*UARTn_Int_RDA[2])(void);
    //THRE Interrupt: Status of this can be read from LSR[5]

    //RXLS Interrupt: UARTn RX Line Status Interrupts. Status can be read from LSR[4:1]

public:
    //Declare queue for receiving the data
    QueueHandle_t rx_fifo;
    /**
     * Peripheral: UART2 and UART3 enumerations
     */
    enum Peripheral
    {
        UART2 = 0,
        UART3 = 1
    };
    enum En_Interrupt
    {
        INT_DISABLED = 0,
        INT_ENABLED = 1
    };
    /* const m_uart_pc: used to hold the uart peripheral channel selected by the user
     * , gets used in member functions*/
    const uint32_t m_uart_pc;

    /**
     * m_uart_int: used to hold flag for whether UART Interrupts are being used
     */
    bool m_uart_int;

    enum WordLengthSelect
    {
        CHAR_LENGTH_5BIT = 0,
        CHAR_LENGTH_6BIT = 1,
        CHAR_LENGTH_7BIT = 2,
        CHAR_LENGTH_8BIT = 3
    };

    /**
     * Sets the UART Peripheral Channel
     * @param uart_pc: UART Peripheral Channel set will initialize private
     * member 'm_uart_pc'
     */
    UARTDriver(uint32_t uart_pc, bool uart_int);
    /**
     * Initializes UART Peripheral x
     * @param baud_rate: set baud rate for uart channel
     * @param item_size: set the number of bits that'll be transferred
     */
    bool init(uint32_t baud_rate, WordLengthSelect item_size);
    /**
     * Transfers word length of char at the baud rate specified
     * @param send: char data that'll be sent
     */
    void transfer(char send);
    /**
     * Receives char data at the baud rate specified
     */
    char receive();

    /**
     * Extra Credit: Interrupt Functions
     * During UnIIR access, interrupts are frozen
     */

    /**
     * Interrupt Driven Receive function
     */
    void intReceive();

    /**
     * This handler will place a function pointer within the lookup table for the
     * externalIRQHandler to find.
     *
     * @param[in] port          specify the UART port
     * @param[in] pin           specify the UART pin to assign an ISR to
     * @param[in] pin_isr       function to run when interrupt event occurs
     * @param[in] condition     condition for the interrupt to occur on: RISING, FALLING, BOTH edges
     * @return returns if valid ports, pins, isrs were supplied and pin isr insertion was successful
     */
    bool attachUARTInterruptHandler(uint8_t port, uint8_t pin, void (*pin_isr)(void), UARTInterruptCondition_E condition);

    /**
     * This function is invoked by the CPU (through c_uartx_handler) asynchronously when a Port/Pin
     * interrupt occurs. This function is where you will check the uartx interrupt status, such as IIR,
     * and then invoke the user's registered callback for the appropriate interrupt: .
     *
     * VERY IMPORTANT!
     *  - Be sure to clear the interrupt flag that caused this interrupt, or this function will be called
     *    repetitively and lock your system.
     *  - NOTE that your code needs to be able to handle two UART2 and UART3 interrupts.
     */
    void handle_interrupt(void);

    virtual ~UARTDriver();
};

#endif /* UARTDRIVER_HPP_ */
