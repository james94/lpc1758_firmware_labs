/*
 * LabGPIO.hpp
 *
 *  Created on: Jan 30, 2018
 *      Author: james
 */

#ifndef LABGPIO_HPP_
#define LABGPIO_HPP_

#include "LPC17xx.h"
#include "uart0_min.h"
#include "stdbool.h"

class LabGPIO
{
protected:
    /**
     * port, pin and any other variables should be placed here.
     * NOTE: that the state of the pin should not be recorded here. The true
     *      source of that information is embedded in the hardware registers
     */
    uint8_t m_port, m_pin;
public:
    enum PIN_DIRECTION
    {
        INPUT = 0,
        OUTPUT = 1
    };
    enum PIN_LEVEL
    {
        LOW = 0,
        HIGH = 1
    };
    /**
     * Does not modify any hardware registers at this point
     * Only stores the port and pin using the constructor.
     *
     * @param {uint8_t} port - port number from 0 to 4
     * @param {uint8_t} pin  - pin number from 0 to 31
     */
    LabGPIO(uint8_t port, uint8_t pin);
    /**
     * Alters the hardware registers to set the pin as an input
     */
    void setAsInput();
    /**
     * Alters the hardware registers to set the pin as an output
     */
    void setAsOutput();
    /**
     * Alters "the set" direction as output or input depending on the
     * input argument to this function
     *
     * @param {bool} output - true => output, false => set pin to input
     */
    void setDirection(bool output);
    /**
     * Alters the hardware registers to set the pin as high
     */
    void setHigh();
    /**
     * Alters the hardware registers to set the pin as low
     */
    void setLow();
    /**
     * Alters the hardware registers to set the pin as low
     *
     * @param {bool} high - true => pin high, false => pin low
     */
    void set(bool high);
    /**
     * Returns the state of the pin (input or output, doesn't matter)
     *
     * @return {bool} level of pin high => true, low => false
     */
    bool getLevel();

    ~LabGPIO();
};

#endif /* LABGPIO_HPP_ */
