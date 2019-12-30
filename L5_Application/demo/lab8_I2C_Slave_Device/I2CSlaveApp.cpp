/*
 * I2CSlaveApp.cpp
 *
 *  Created on: Apr 5, 2018
 *      Author: james
 */

#include <demo/lab8_I2C_Slave_Device/I2CSlaveApp.hpp>

void runI2CMonitorRegisters(void)
{
    //Slave will act as a controller
    I2C2& i2c = I2C2::getInstance(); // Get I2C driver instance
    const uint8_t slaveAddr = 0xC0;  // Pick any address other than an existing one at i2c2.hpp
    uint8_t reg[256] = { 0 }; // Our slave read/write reg (This is the memory your other master board will read/write)

    // I2C is already initialized before main(), so you will have to add initSlave() to i2c base class for your slave driver
    i2c.initSlave(slaveAddr, reg, sizeof(reg));

    // I2C interrupt will (should) modify our reg.
    // So just monitor our reg, and print and/or light up LEDs
    // ie: If reg[0] == 0, then LED ON, else LED OFF
    uint8_t prev[256];

    for(int init = 0; init < 256; ++init)
    {
        prev[init] = reg[init];
    }

    while(1)
    {
        for(int compare = 0; compare < 256; ++compare)
        {
            if (prev[compare] != reg[compare]) {
                u0_dbg_printf("prev[%i] was %x\n", compare, prev[compare]);
                prev[compare] = reg[compare];
                u0_dbg_printf("reg[%i] changed to %#x by the other Master Board\n", compare, reg[compare]);
            }
        }
    }
    vTaskStartScheduler();
}
