/*
 * SSPDriver.cpp
 *
 *  Created on: Mar 6, 2018
 *      Author: james
 */

#include <demo/lab5_SSP/SSPDriver.hpp>

constexpr LPC_SSP_TypeDef * SSPDriver::SSP[];

SSPDriver::SSPDriver()
{
    // TODO Auto-generated constructor stub

    m_peripheral = SSP1;
}

/**
 * 1) Powers on SSPn peripheral
 * 2) Set peripheral clock
 * 3) Sets pins for specified peripheral to MOSI, MISO, and SCK
 *
 * @param peripheral which peripheral SSP0 or SSP1 you want to select.
 * @param data_size_select transfer size data width; To optimize the code, look for a pattern in the datasheet
 * @param format is the code format for which synchronous serial protocol you want to use.
 * @param divide is the how much to divide the clock for SSP; take care of error cases such as the value of 0, 1, and odd numbers
 *
 * @return true if initialization was successful
 */
bool SSPDriver::init(Peripheral peripheral, uint8_t data_size_select, FrameModes format, uint8_t divide)
{
    m_peripheral = peripheral;
    bool init_status = false;
    switch(m_peripheral)
    {
        case SSP0:
            // 1) Powers on SSPn peripheral: PCSSP0 bit 21 (Table 46)
            LPC_SC->PCONP |= (1 << 21);
            // 2) Set peripheral clock = cclk: Select PCLK_SSP0 bits 11_10 in PCLKSEL1 (4.7.3)
            LPC_SC->PCLKSEL1 |= (1 << 10);

            // 3) Sets pins for specified peripheral to MOSI, MISO, and SCK (8.5)
            LPC_PINCON->PINSEL3 &= ~( (3 << 8) | (3 << 14) | (3 << 16) );
            LPC_PINCON->PINSEL3 |= (3 << 8); //P1.20 (SCK0)
            LPC_PINCON->PINSEL3 |= (3 << 14); //P1.23 (MISO0)
            LPC_PINCON->PINSEL3 |= (3 << 16); //P1.24 (MOSI0)

            // 4) Config basic op of SSP0 Controller with SSP0CR0 reg (Table 371)
            SSP[m_peripheral]->CR0 |= (data_size_select); //DSS controls number of bits tx per frame
            SSP[m_peripheral]->CR0 |= (format << 4); // Select frame format for Synchronous Serial Port

            // 5) Config certain parts of op of SSP0 Controller with SSP0CR1 reg
            SSP[m_peripheral]->CR1 |= (1 << 1); // SSP Enable: SSP Controller interacts with other devices on serial bus

            // 2.1) In Master Mode, set Prescaler value to 'divide' peripheral clock to yield Prescaler clock (18.6.5)
            SSP[m_peripheral]->CPSR |= (divide);
            init_status = true;
            break;
        case SSP1:
            // 1) Powers on SSP1 peripheral: PCSSP1 bit 10 (Table 46)
            LPC_SC->PCONP |= (1 << 10);
            // 2) Set peripheral clock = cclk: Select PCLK_SSP1 bits 21_20 in PCLKSEL0 (4.7.3)
            LPC_SC->PCLKSEL0 |= (1 << 20);

            // 3) Sets pins for specified peripheral to MOSI, MISO, and SCK (8.5)
            LPC_PINCON->PINSEL0 &= ~( (3 << 12) | (3 << 14) | (3 << 16) | (3 << 18) ); // Clr P0.6-9
            LPC_PINCON->PINSEL0 |= (2 << 14); // Set P0.7 (SCK1)
            LPC_PINCON->PINSEL0 |= (2 << 16); // Set P0.8 (MISO1)
            LPC_PINCON->PINSEL0 |= (2 << 18); // Set P0.9 (MOSI1)

            // 4) Config basic op of SSP0 Controller with SSP0CR0 reg (Table 371)
            SSP[m_peripheral]->CR0 |= (data_size_select); //DSS controls number of bits tx per frame
            SSP[m_peripheral]->CR0 |= (format << 4); // Select frame format for Synchronous Serial Port

            // 5) Config certain parts of op of SSP0 Controller with SSP0CR1 reg
            SSP[m_peripheral]->CR1 |= (1 << 1); // SSP Enable: SSP Controller interacts with other devices on serial bus

            // 2.1) In Master Mode, set Prescaler value to 'divide' peripheral clock to yield Prescaler clock (18.6.5)
            SSP[m_peripheral]->CPSR |= (divide);


            init_status = true;
            break;
        default:
            u0_dbg_printf("Init Failed. Invalid SSPn peripheral\n");
            init_status = false;
            break;
    }



    return init_status;
}


/**
 * Transfers a byte via SSP to an external device using the SSP data register.
 * This region must be protected by a mutex static to this class.
 *
 * @return received byte from external device via SSP data register
 */
uint8_t SSPDriver::transfer(uint8_t send)
{
    // As the master, writes will send a frame of 8 bits to slave device
    LPC_SSP1->DR = send;


    // wait until SSPn controller is done sending/receiving a frame
    while(LPC_SSP1->SR & (1 << 4));


    // capture data from slave device
    return LPC_SSP1->DR;
}

