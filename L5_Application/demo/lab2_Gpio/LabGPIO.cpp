/*
 * LabGPIO.cpp
 *
 *  Created on: Jan 30, 2018
 *      Author: james
 */

#include <demo/lab2_Gpio/LabGPIO.hpp>

//Stores the port and pin that the user selects
LabGPIO::LabGPIO(uint8_t port, uint8_t pin)
{
    //https://stackoverflow.com/questions/1238613/what-is-the-difference-between-the-dot-operator-and-in-c
    //When to use "." vs "->"?
    //If you need to access property of an object or object ref, use "."
    //If you access property of obj thru pointer, use "->" or "(*)."
    (*this).m_port = port;
    this->m_pin = pin;
}

/**
 * Alters hardware registers to set pin as an input
 */
void LabGPIO::setAsInput()
{
    //According to UM10360 9.5.1,
    //FIODIR reg controls the GPIO port pin direction
    //0 in bit pos X of reg, sets pin X as input
    if(m_port == 0)
    {
        LPC_GPIO0->FIODIR &= ~(1 << m_pin);
    }
    else if(m_port == 1)
    {
        LPC_GPIO1->FIODIR &= ~(1 << m_pin);
    }
    else if(m_port == 2)
    {
        LPC_GPIO2->FIODIR &= ~(1 << m_pin);
    }
    else if(m_port == 3)
    {
        LPC_GPIO3->FIODIR &= ~(1 << m_pin);
    }
    else if(m_port == 4)
    {
        LPC_GPIO4->FIODIR &= ~(1 << m_pin);
    }
    else
    {
        uart0_puts("Invalid port");
    }
}

/**
 * Alters hardware registers to set pin as an output
 */
void LabGPIO::setAsOutput()
{
    //According to UM10360,
    //FIODIR reg controls the GPIO port pin direction
    //1 in bit pos X of reg, sets pin X as output
    if(m_port == 0)
    {
        LPC_GPIO0->FIODIR |= (1 << m_pin);
    }
    else if(m_port == 1)
    {
        LPC_GPIO1->FIODIR |= (1 << m_pin);
    }
    else if(m_port == 2)
    {
        LPC_GPIO2->FIODIR |= (1 << m_pin);
    }
    else if(m_port == 3)
    {
        LPC_GPIO3->FIODIR |= (1 << m_pin);
    }
    else if(m_port == 4)
    {
        LPC_GPIO4->FIODIR |= (1 << m_pin);
    }
    else
    {
        uart0_puts("Invalid port");
    }
}

/**
 * Alters "the set" direction as output or input depending on the
 * input argument to this function
 *
 */
void LabGPIO::setDirection(bool output)
{
    if(output == true)
    {
        setAsOutput();
    }
    else if(output == false)
    {
        setAsInput();
    }
}

/**
 * Alters the hardware registers to set the pin as high
 */
void LabGPIO::setHigh()
{
    //According to UM10360 9.5.4,
    //FIOPIN reg controls the GPIO port pin voltage level
    //1 in bit pos X of reg, sets the output reg value to HIGH
    switch(m_port)
    {
        case 0:
            LPC_GPIO0->FIOPIN |= (1 << m_pin);
            break;
        case 1:
            LPC_GPIO1->FIOPIN |= (1 << m_pin);
            break;
        case 2:
            LPC_GPIO2->FIOPIN |= (1 << m_pin);
            break;
        case 3:
            LPC_GPIO3->FIOPIN |= (1 << m_pin);
            break;
        case 4:
            LPC_GPIO4->FIOPIN |= (1 << m_pin);
            break;
        default:
            uart0_puts("Invalid port");
            break;
    }
}

/**
 * Alters the hardware registers to set the pin as low
 */
void LabGPIO::setLow()
{
    //According to UM10360 9.5.4,
    //FIOPIN reg controls the GPIO port pin voltage level
    //0 in bit pos X of reg, sets the output reg value to LOW
    switch(m_port)
    {
        case 0:
            LPC_GPIO0->FIOPIN &= ~(1 << m_pin);
            break;
        case 1:
            LPC_GPIO1->FIOPIN &= ~(1 << m_pin);
            break;
        case 2:
            LPC_GPIO2->FIOPIN &= ~(1 << m_pin);
            break;
        case 3:
            LPC_GPIO3->FIOPIN &= ~(1 << m_pin);
            break;
        case 4:
            LPC_GPIO4->FIOPIN &= ~(1 << m_pin);
            break;
        default:
            uart0_puts("Invalid port");
            break;
    }
}

/**
 * Alters the hardware registers to set the pin as low or high
 *
 * @param {bool} high - true => pin high, false => pin low
 */
void LabGPIO::set(bool high)
{
    switch((int)high)
    {
        case 1:
            setHigh();
            break;
        case 0:
            setLow();
            break;
        default:
            uart0_puts("Invalid signal");
            break;
    }
}

/**
 * Returns the state of the pin (input or output, doesn't matter)
 *
 * @return {bool} level of pin high => true, low => false
 */
bool LabGPIO::getLevel()
{
    //Reads Port Pin's value and stores into pin_state
    bool pin_state = false;
    switch(m_port)
    {
        case 0:
            pin_state = (LPC_GPIO0->FIOPIN & (1 << m_pin));
            break;
        case 1:
            pin_state = (LPC_GPIO1->FIOPIN & (1 << m_pin));
            break;
        case 2:
            pin_state = (LPC_GPIO2->FIOPIN & (1 << m_pin));
            break;
        case 3:
            pin_state = (LPC_GPIO3->FIOPIN & (1 << m_pin));
            break;
        case 4:
            pin_state = (LPC_GPIO4->FIOPIN & (1 << m_pin));
            break;
        default:
            uart0_puts("Invalid port");
            break;
    }

    return pin_state;
}

LabGPIO::~LabGPIO()
{
    // TODO Auto-generated destructor stub
}

