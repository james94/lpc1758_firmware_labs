/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

#include <string.h>         // memcpy

#include "i2c_base.hpp"
#include "lpc_sys.h"
#include "LPC17xx.h"
#include "printf_lib.h"


/**
 * Instead of using a dedicated variable for read vs. write, we just use the LSB of
 * the user address to indicate read or write mode.
 */
#define I2C_SET_READ_MODE(addr)     (addr |= 1)     ///< Set the LSB to indicate read-mode
#define I2C_SET_WRITE_MODE(addr)    (addr &= 0xFE)  ///< Reset the LSB to indicate write-mode
#define I2C_READ_MODE(addr)         (addr & 1)      ///< Read address is ODD
#define I2C_WRITE_ADDR(addr)        (addr & 0xFE)   ///< Write address is EVEN
#define I2C_READ_ADDR(addr)         (addr | 1)      ///< Read address is ODD



void I2C_Base::handleInterrupt()
{
    /* If transfer finished (not busy), then give the signal */
    if (busy != i2cStateMachine()) {
        long higherPriorityTaskWaiting = 0;
        xSemaphoreGiveFromISR(mTransferCompleteSignal, &higherPriorityTaskWaiting);
        portEND_SWITCHING_ISR(higherPriorityTaskWaiting);
    }
}

uint8_t I2C_Base::readReg(uint8_t deviceAddress, uint8_t registerAddress)
{
    uint8_t byte = 0;
    readRegisters(deviceAddress, registerAddress, &byte, 1);
    return byte;
}

bool I2C_Base::readRegisters(uint8_t deviceAddress, uint8_t firstReg, uint8_t* pData, uint32_t bytesToRead)
{
    I2C_SET_READ_MODE(deviceAddress);
    return transfer(deviceAddress, firstReg, pData, bytesToRead);
}

bool I2C_Base::writeReg(uint8_t deviceAddress, uint8_t registerAddress, uint8_t value)
{
    return writeRegisters(deviceAddress, registerAddress, &value, 1);
}

bool I2C_Base::writeRegisters(uint8_t deviceAddress, uint8_t firstReg, uint8_t* pData, uint32_t bytesToWrite)
{
    I2C_SET_WRITE_MODE(deviceAddress);
    return transfer(deviceAddress, firstReg, pData, bytesToWrite);
}

bool I2C_Base::transfer(uint8_t deviceAddress, uint8_t firstReg, uint8_t* pData, uint32_t transferSize)
{
    bool status = false;
    if(mDisableOperation || !pData) {
        return status;
    }

    // If scheduler not running, perform polling transaction
    if(taskSCHEDULER_RUNNING != xTaskGetSchedulerState())
    {
        //Entry point for I2C Transaction
        i2cKickOffTransfer(deviceAddress, firstReg, pData, transferSize);

        // Wait for transfer to finish
        const uint64_t timeout = sys_get_uptime_ms() + I2C_TIMEOUT_MS;
        while (!xSemaphoreTake(mTransferCompleteSignal, 0)) {
            if (sys_get_uptime_ms() > timeout) {
                break;
            }
        }

        status = (0 == mTransaction.error);
    }
    else if (xSemaphoreTake(mI2CMutex, OS_MS(I2C_TIMEOUT_MS)))
    {
        // Clear potential stale signal and start the transfer
        xSemaphoreTake(mTransferCompleteSignal, 0);
        i2cKickOffTransfer(deviceAddress, firstReg, pData, transferSize);

        // Wait for transfer to finish and copy the data if it was read mode
        if (xSemaphoreTake(mTransferCompleteSignal, OS_MS(I2C_TIMEOUT_MS))) {
            status = (0 == mTransaction.error);
        }

        xSemaphoreGive(mI2CMutex);
    }

    return status;
}

bool I2C_Base::checkDeviceResponse(uint8_t deviceAddress)
{
    uint8_t dummyReg = 0;
    uint8_t notUsed = 0;

    // The I2C State machine will not continue after 1st state when length is set to 0
    uint32_t lenZeroToTestDeviceReady = 0;

    return readRegisters(deviceAddress, dummyReg, &notUsed, lenZeroToTestDeviceReady);
}

I2C_Base::I2C_Base(LPC_I2C_TypeDef* pI2CBaseAddr) :
        mpI2CRegs(pI2CBaseAddr),
        mDisableOperation(false)
{
    mI2CMutex = xSemaphoreCreateMutex();
    mTransferCompleteSignal = xSemaphoreCreateBinary();

    // Optional: Provide names of the FreeRTOS objects for the Trace Facility
    vTraceSetMutexName(mI2CMutex, "I2C Mutex");
    vTraceSetSemaphoreName(mTransferCompleteSignal, "I2C Finish Sem");

    if((unsigned int)mpI2CRegs == LPC_I2C0_BASE)
    {
        mIRQ = I2C0_IRQn;
    }
    else if((unsigned int)mpI2CRegs == LPC_I2C1_BASE)
    {
        mIRQ = I2C1_IRQn;
    }
    else if((unsigned int)mpI2CRegs == LPC_I2C2_BASE)
    {
        mIRQ = I2C2_IRQn;
    }
    else {
        mIRQ = (IRQn_Type)99; // Using invalid IRQ on purpose
    }
}

//Initializes I2C Communication Bus
bool I2C_Base::init(uint32_t pclk, uint32_t busRateInKhz)
{
    // Power on I2C
    switch(mIRQ) {
        case I2C0_IRQn: lpc_pconp(pconp_i2c0, true);  break;
        case I2C1_IRQn: lpc_pconp(pconp_i2c1, true);  break;
        case I2C2_IRQn: lpc_pconp(pconp_i2c2, true);  break;
        default: return false;
    }

    //why need clear all I2C Flags?  To clear SI bit
    mpI2CRegs->I2CONCLR = 0x6C;           // Clear ALL I2C Flags

    /**
     * Per I2C high speed mode:
     * HS mode master devices generate a serial clock signal with a HIGH to LOW ratio of 1 to 2.
     * So to be able to optimize speed, we use different duty cycle for high/low
     *
     * Compute the I2C clock dividers.
     * The LOW period can be longer than the HIGH period because the rise time
     * of SDA/SCL is an RC curve, whereas the fall time is a sharper curve.
     */
    const uint32_t percent_high = 40;
    const uint32_t percent_low = (100 - percent_high);
    const uint32_t freq_hz = (busRateInKhz > 1000) ? (100 * 1000) : (busRateInKhz * 1000);
    const uint32_t half_clock_divider = (pclk / freq_hz) / 2;
    mpI2CRegs->I2SCLH = (half_clock_divider * percent_high) / 100;
    mpI2CRegs->I2SCLL = (half_clock_divider * percent_low ) / 100;

    // Set I2C slave address and enable I2C
    mpI2CRegs->I2ADR0 = 0;
    mpI2CRegs->I2ADR1 = 0;
    mpI2CRegs->I2ADR2 = 0;
    mpI2CRegs->I2ADR3 = 0;

    // Enable I2C and the interrupt for it
    // Before master mode can be entered, I2CONSET reg must be initialized 19.6.1
    // I2EN=1 enables I2C function, AA=0 master can't enter slave mode
    // STA, STO and SI all = 0
    mpI2CRegs->I2CONSET = 0x40;
    NVIC_EnableIRQ(mIRQ);

    return true;
}

// Initializes I2C Slave Device on Communication BUS (UM10360)
bool I2C_Base::initSlave(uint8_t slaveAddr, uint8_t *pBufferData, uint32_t bufferSize)
{
    //Setup slaves data to be R/W in I2C Transaction
    mSlaveTransaction.slaveAddr = slaveAddr;
    pRegMap = pBufferData;
    mSlaveCounter = bufferSize;

    u0_dbg_printf("slaveAddr is: %x\n", mSlaveTransaction.slaveAddr);

    //1. Power I2C Peripheral for I2C Interface 2
    LPC_SC->PCONP |= (1 << 26);

    //3. Pins
    //3.1 Select I2C2 via PINSEL0 reg
    LPC_PINCON->PINSEL0 &= ~(0xF << 20); //reset P0.10/P0.11 function
    LPC_PINCON->PINSEL0 |= (0xA << 20); //set P0.10/P0.11 as SDA2 and SCL2
    //3.2 Select pin modes for port pins with I2C2 functions through PINMODE reg
    //Enable pull-up, pull-down resistors on P0.10/P0.11
    LPC_PINCON->PINMODE0 &= ~(0xF << 20);
    //Disable pull-up, pull-down resistors on P0.10/P0.11
    LPC_PINCON->PINMODE0 |= (0xA << 20);
    //Select Open Drain mode on P0.10/P0.11
    const uint32_t i2c_pin_mask = (( 1 << 10 | 1 << 11 ));
    LPC_PINCON->PINMODE_OD0 |= i2c_pin_mask;

    //4. Enable I2C2 Interrupts
    NVIC_EnableIRQ(I2C2_IRQn);

    //5. Initialize I2C Interface as Slave (19.10.1)

    //Write 0x44 to I2CONSET reg to set I2EN and AA bits (Enable Slave mode)
    mpI2CRegs->I2CONSET = 0x44;
    //Set I2C slave address inside I2ADR[0] reg
    mpI2CRegs->I2ADR0 = mSlaveTransaction.slaveAddr;
    mpI2CRegs->I2ADR1 = 0;
    mpI2CRegs->I2ADR2 = 0;
    mpI2CRegs->I2ADR3 = 0;

    return true;
}


/// Private ///

void I2C_Base::i2cKickOffTransfer(uint8_t devAddr, uint8_t regStart, uint8_t* pBytes, uint32_t len)
{
    mTransaction.error     = 0;
    mTransaction.slaveAddr = devAddr;
    mTransaction.firstReg  = regStart;
    mTransaction.trxSize   = len;
    mTransaction.pMasterData   = pBytes;

    // Send START, I2C State Machine will finish the rest.
    mpI2CRegs->I2CONSET = 0x20;
}

/*
 * I2CONSET bits
 * 0x04 AA
 * 0x08 SI
 * 0x10 STOP
 * 0x20 START
 * 0x40 ENABLE
 *
 * I2CONCLR bits
 * 0x04 AA
 * 0x08 SI
 * 0x20 START
 * 0x40 ENABLE
 */
I2C_Base::mStateMachineStatus_t I2C_Base::i2cStateMachine()
{
    enum {
        // General states :
        busError        = 0x00,
        start           = 0x08,
        repeatStart     = 0x10,
        arbitrationLost = 0x38,

        // Master Transmitter States:
        slaveAddressAcked  = 0x18,
        slaveAddressNacked = 0x20,
        dataAckedBySlave   = 0x28,
        dataNackedBySlave  = 0x30,

        // Master Receiver States:
        readAckedBySlave      = 0x40,
        readModeNackedBySlave = 0x48,
        dataAvailableAckSent  = 0x50,
        dataAvailableNackSent = 0x58,

        // Slave Receiver States:
        writeModeAckedBySlave = 0x60, //Slave acknowledges master is in transmit mode
        writeDataAckedBySlave = 0x80, //Slave acknowledges data transmitted by Master
        writeLastByteAckedBySlave = 0x88, //Slave acknowledges last byte written by Master

        // Slave Transmitter States:
        readModeAckedBySlave = 0xA8, //Slave acknowledges master is in receive mode
        writeDataAckedByMaster = 0xB8, //Slave transmits data to master
        writeLastByteAckedByMaster = 0xC0, //Slave transmits last byte to master

        // Either Slave State:
        rxStopOrRStartAsASlave = 0xA0
    };

    mStateMachineStatus_t state = busy;

    /*
     ***********************************************************************************************************
     * Write-mode state transition :
     * start --> slaveAddressAcked --> dataAckedBySlave --> ... (dataAckedBySlave) --> (stop)
     *
     * Read-mode state transition :
     * start --> slaveAddressAcked --> dataAcked --> repeatStart --> readAckedBySlave
     *  For 2+ bytes:  dataAvailableAckSent --> ... (dataAvailableAckSent) --> dataAvailableNackSent --> (stop)
     *  For 1  byte :  dataAvailableNackSent --> (stop)
     ***********************************************************************************************************
     */

    /* Me being lazy and using #defines instead of inline functions :( */
    #define clearSIFlag()       mpI2CRegs->I2CONCLR = (1<<3)
    #define setSTARTFlag()      mpI2CRegs->I2CONSET = (1<<5)
    #define clearSTARTFlag()    mpI2CRegs->I2CONCLR = (1<<5)
    #define setAckFlag()        mpI2CRegs->I2CONSET = (1<<2)
    #define setNackFlag()       mpI2CRegs->I2CONCLR = (1<<2)

    /* yep ... lazy again */
    #define setStop()           clearSTARTFlag();                           \
                                mpI2CRegs->I2CONSET = (1<<4);               \
                                clearSIFlag();                              \
                                while((mpI2CRegs->I2CONSET&(1<<4)));        \
                                if(I2C_READ_MODE(mTransaction.slaveAddr))   \
                                    state = readComplete;                   \
                                else                                        \
                                    state = writeComplete;

    switch (mpI2CRegs->I2STAT)
    {
        case start:// 0x08
            mpI2CRegs->I2DAT = I2C_WRITE_ADDR(mTransaction.slaveAddr);
            clearSIFlag();
            break;
        case repeatStart: //0x10
            mpI2CRegs->I2DAT = I2C_READ_ADDR(mTransaction.slaveAddr);
            clearSIFlag();
            break;

        case slaveAddressAcked: //0x18
            clearSTARTFlag();
            // No data to transfer, this is used just to test if the slave responds
            if(0 == mTransaction.trxSize) {
                setStop();
            }
            else {
                mpI2CRegs->I2DAT = mTransaction.firstReg;
                clearSIFlag();
            }
            break;

        case dataAckedBySlave: //0x28
            if (I2C_READ_MODE(mTransaction.slaveAddr)) {
                setSTARTFlag(); // Send Repeat-start for read-mode
                clearSIFlag();
            }
            else {
                if(0 == mTransaction.trxSize) {
                    setStop();
                }
                else {
                    mpI2CRegs->I2DAT = *(mTransaction.pMasterData);
                    ++mTransaction.pMasterData;
                    --mTransaction.trxSize;
                    clearSIFlag();
                }
            }
            break;

        /* In this state, we are about to initiate the transfer of data from slave to us
         * so we are just setting the ACK or NACK that we'll do AFTER the byte is received.
         */
        case readAckedBySlave: // 0x40
            clearSTARTFlag();
            if(mTransaction.trxSize > 1) {
                setAckFlag();  // 1+ bytes: Send ACK to receive a byte and transition to dataAvailableAckSent
            }
            else {
                setNackFlag();  //  1 byte : NACK next byte to go to dataAvailableNackSent for 1-byte read.
            }
            clearSIFlag();
            break;
        case dataAvailableAckSent: //0x50
            *mTransaction.pMasterData = mpI2CRegs->I2DAT;
            ++mTransaction.pMasterData;
            --mTransaction.trxSize;

            if(1 == mTransaction.trxSize) { // Only 1 more byte remaining
                setNackFlag();// NACK next byte --> Next state: dataAvailableNackSent
            }
            else {
                setAckFlag(); // ACK next byte --> Next state: dataAvailableAckSent(back to this state)
            }

            clearSIFlag();
            break;
        case dataAvailableNackSent: // 0x58: Read last-byte from Slave
            *mTransaction.pMasterData = mpI2CRegs->I2DAT;
            setStop();
            break;

        case arbitrationLost: // 0x38
            // We should not issue stop() in this condition, but we still need to end our  transaction.
            state = I2C_READ_MODE(mTransaction.slaveAddr) ? readComplete : writeComplete;
            mTransaction.error = mpI2CRegs->I2STAT;
            break;

            /*
             ***********************************************************************************************************
             * SLAVE STATES in respect to master states
             * Master Write-mode state transition in association with Slave Read-mode transition :
             * Master        start          -->  slaveAddressAcked --> dataAckedBySlave          --> ... (dataAckedBySlave) --> (stop)
             * Slave    writeModeAckedBySlave --> writeDataAckedBySlave --> ... (writeDataAckedBySlave) --> writeLastByteAckedBySlave
             *
             * Master Read-mode state transition in association with Slave Write-mode transition :
             * start --> slaveAddressAcked --> dataAcked --> repeatStart --> readAckedBySlave
             *  For 2+ bytes:  dataAvailableAckSent --> ... (dataAvailableAckSent) --> dataAvailableNackSent --> (stop)
             *  For 1  byte :  dataAvailableNackSent --> (stop)
             * Slave                                                    readModeAckedBySlave -->
             *                 writeDataAckedByMaster --> ... (writeDataAckedByMaster) -->
             *                 writeLastByteAckedByMaster
             ***********************************************************************************************************
             */
            // Slave Receive Mode
        case writeModeAckedBySlave: // 0x60
            //Own Slave Address + Write has been received
            //Verify SLA+W in I2DAT matches actual SLA
            u0_dbg_printf("State: 0x60: RX Own SLA+W, ACK Returned, Entered Slave RX MODE\n");
            //1. set AA flag
            setAckFlag();
            //2. clear SI flag
            clearSIFlag();
            //3. Set up Slave Receive mode data buffer for I2C Transaction
            //3.1 Slave SAVES the index of the Register Master wants to READ
            mSlaveTransaction.slaveFirstReg = mpI2CRegs->I2DAT;
            //3.1 Obtain pSlaveData[FirstReg] or register user wants to R or W to
            mSlaveTransaction.pSlaveData = pRegMap + mSlaveTransaction.slaveFirstReg;
            //4. Initialize Slave data counter for I2C Transaction (Size of Buffer = 5)
            mSlaveTransaction.slaveTrxSize = mSlaveCounter - mSlaveTransaction.slaveFirstReg;
            u0_dbg_printf("Next: DATA will be RX, then ACK will be Returned, onto 0x80\n");
            break;

        case writeDataAckedBySlave: // 0x80
            u0_dbg_printf("State: 0x80: DATA RX, ACK Returned\n");
            //1. Read data byte from I2DAT into Slave Receive Buffer
            *mSlaveTransaction.pSlaveData = mpI2CRegs->I2DAT;
            //2. Decrement Slave Data Counter, if not last byte, skip to 5.
            --mSlaveTransaction.slaveTrxSize;
            //Check I2C Transaction Size, if not 1, then Slave will ACK Data from Master
            if(mSlaveTransaction.slaveTrxSize == 1)
            {
                //3. Clear SI Flag and set Nack Flag
                clearSIFlag();
                setNackFlag();
                u0_dbg_printf("Next: Last DATA Byte will be RX, then NACK will Be Returned, onto 0x88\n");
                //4. exit
                break;
            }
            else
            {
                //5. Set Ack Flag
                setAckFlag();
                //6. clear SI Flag
                clearSIFlag();
                //7. increment slave receive buffer pointer
                ++mSlaveTransaction.pSlaveData;
                u0_dbg_printf("Next: DATA will be RX, then ACK will Be Returned, onto 0x80\n");
                //8. exit
                break;
            }

        case writeLastByteAckedBySlave: // 0x88
            //Received data won't be saved cause NOT ACK has been returned
            u0_dbg_printf("State: 0x88: Last DATA Byte RX, NACK Returned\n");
            u0_dbg_printf("Switched to NOT ADDR Slave Mode, RX DATA won't be Saved\n");
            //1. Set Ack Flag
            setAckFlag();
            //2. Clear SI Flag
            clearSIFlag();
            u0_dbg_printf("Onto 0xA0\n");
            break;

            // Slave Transmitter Mode
        case readModeAckedBySlave: // 0xA8 (Needed, so Slave knows Master is in READ MODE)
            u0_dbg_printf("State: 0xA8: Slave ACK Master in READ MODE\n");
            //Own SLA+R has been received, ACK has been returned.
            //Data will be transmitted, ACK will be received.
            //1. Load I2DAT from Slave Transmit Buffer with First Data Byte
            mpI2CRegs->I2DAT = *mSlaveTransaction.pSlaveData;
            //2. Set Ack Flag
            setAckFlag();
            //3. Clear SI Flag
            clearSIFlag();
            //4. Set up Slave Transmit Mode Data Buffer
            //4.1 Slave Reads Index of Register Master Wants to Read
            mSlaveTransaction.slaveFirstReg = mpI2CRegs->I2DAT;
            //4.2 Update Array Element Location of Register Pointer Points to
            mSlaveTransaction.pSlaveData = pRegMap + mSlaveTransaction.slaveFirstReg;
            // Initialize slave data counter
            mSlaveTransaction.slaveTrxSize = mSlaveCounter - mSlaveTransaction.slaveFirstReg;
            //5. Increment Slave TX Buffer Pointer
            ++mSlaveTransaction.pSlaveData;
            u0_dbg_printf("Next: DATA Byte will be TX, then ACK will be RX, onto 0xB8\n");
            break;

        case writeDataAckedByMaster: // 0xB8 (Needed, so slave TX Data to Master)
            u0_dbg_printf("State: 0xB8: Master ACKED Slave's TX DATA\n");
            //Data has been transmitted, ACK has been received
            //1. Load I2DAT with Slave Transmit Buffer Register Byte
            mpI2CRegs->I2DAT = *mSlaveTransaction.pSlaveData;
            //4. Increment Slave Transmit Buffer Pointer
            ++mSlaveTransaction.pSlaveData;
            --mSlaveTransaction.slaveTrxSize;
            if(mSlaveTransaction.slaveTrxSize == 1)
            {
                setNackFlag(); //write last byte and NACK
                //u0_dbg_printf("Next: Last DATA Byte will be TX, then NACK will be TX, onto 0xC0");
            }
            else
            {
                setAckFlag(); //write more bytes and ACK
                //u0_dbg_printf("Next: DATA Byte will be TX, then ACK will be RX, onto 0xB8\n");
            }
            //3. Clear SI Flag
            clearSIFlag();
            break;

        case writeLastByteAckedByMaster: // 0xC0 (Needed, so slave stops TX Data)
            u0_dbg_printf("State: 0xC0: Master NACKED Slave's TX DATA\n");
            //1. Set Ack Flag
            setAckFlag();
            //2. Clear SI Flag
            clearSIFlag();
            break;

        case rxStopOrRStartAsASlave:// 0xA0 (Needed, so slave can switch modes or stop)
            u0_dbg_printf("State: 0xA0: STOP or R-START RX while ADDR as Slave\n");
            //Set ACK Flag
            setAckFlag();
            //clear SI Flag
            clearSIFlag();
            u0_dbg_printf("Next: DATA won't be SAVED and SW to NOT ADDR Slave Mode\n");
            break;

        // states that could happen in a non-sunny day scenario
        case slaveAddressNacked:    // no break
        case dataNackedBySlave:     // no break
        case readModeNackedBySlave: // no break
        case busError:              // no break
        default:
            mTransaction.error = mpI2CRegs->I2STAT;
            setStop();
            break;
    }

    return state;
}
