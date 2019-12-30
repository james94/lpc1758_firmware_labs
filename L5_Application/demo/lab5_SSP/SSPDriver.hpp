/*
 * SSPDriver.hpp
 *
 *  Created on: Mar 6, 2018
 *      Author: james
 */

#ifndef SSPDRIVER_HPP_
#define SSPDRIVER_HPP_

#include "LPC17xx.h"
#include "stdbool.h"
#include "sys_config.h"
#include "printf_lib.h"


/**
 * Single Dynamic Thread-safe Synchronous Serial Port Driver
 * for two separate ports and communicate with an external
 * SPI Flash Device.
 *
 * Software Features utilized:
 * - Lookup Table Structures
 * - Enumerations
 * - Bit Field and Structure Mapping
 *
 * SSP Peripheral Features:
 * - Synchronous Serial Communication
 * - Master or slave operation
 * - 8 frame FIFOs for both tx and rx
 * - 4 to 16 bit data frame
 * - DMA transfers supported by GPDMA
 *
 * SSP Peripheral Description:
 * - Capable of operation on SPI, 4-wire, or Microwire bus
 * - Can have multiple masters and slaves on the bus
 * - Only 1 master and 1 slave can communicate on bus during data transfer
 * - Data transfers are "full duplex": data flows from master to slave and slave to master
 * - In practice, only 1 of these data flows carry meaningful data
 *
 * SSP Pin Descriptions:
 * - Serial Clock (SCK0/1):
 *    - Synchronizes the transfer of data
 *    - Driven by master/Received by slave
 *    - Default is on active-high
 *    - SCK1 only switches during a data transfer
 *    - Any other time, SSPn interface holds SCKn in inactive state or doesn't drive it
 * - Master In Slave Out (MISO0/1):
 *    - Transfers serial data from slave to master
 *    - As a master, SSPn clocks in serial data from this signal
 *    - As a slave, SSPn outputs serial data on this signal
 *    - In slave mode, if SSPn isn't selected, then SSPn doesn't drive this signal (left in H-Z)
 * - Master Out Slave In (MOSI0/1):
 *    - Transfers serial data from master to slave
 *    - As a master, SSPn outputs serial data on signal
 *    - As a slave, SSPn clocks in serial data on this signal
 */
class SSPDriver
{
private:
    // SSP Register Lookup Table Structure
    // (C++ Primer 5th Edition, Lippman: 7.6 Static Class Members)
    // SSP[] pointer is associated with the SSPDriver class
    // SSP[] pointer is shared by all SSPDriver objects
    // Can initialize static member in-class cause it has constexpr
    static constexpr LPC_SSP_TypeDef * SSP[] = { LPC_SSP0, LPC_SSP1 };
public:
    // Format for which synchronous serial protocol you want to use (Table 371)
    enum FrameModes
    {
        SPI = 0,
        TI = 1,
        MICROWIRE = 2,
        INVALID = 3
    };

    enum DataSizeSelect
    {
        TRANSFER_4_BITS = 3,
        TRANSFER_5_BITS = 4,
        TRANSFER_6_BITS = 5,
        TRANSFER_7_BITS = 6,
        TRANSFER_8_BITS = 7,
        TRANSFER_9_BITS = 8,
        TRANSFER_10_BITS = 9,
        TRANSFER_11_BITS = 10,
        TRANSFER_12_BITS = 11,
        TRANSFER_13_BITS = 12,
        TRANSFER_14_BITS = 13,
        TRANSFER_15_BITS = 14,
        TRANSFER_16_BITS = 15
    };

    enum Peripheral
    {
        SSP0 = 0,
        SSP1 = 1
    };
    Peripheral m_peripheral;

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
    bool init(Peripheral peripheral, uint8_t data_size_select, FrameModes format, uint8_t divide);

    /**
     * Transfers a byte via SSP to an external device using the SSP data register.
     * This region must be protected by a mutex static to this class.
     *
     * @return received byte from external device via SSP data register
     */
    uint8_t transfer(uint8_t send);

    SSPDriver();

};

#endif /* SSPDRIVER_HPP_ */
