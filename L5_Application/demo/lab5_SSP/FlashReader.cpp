/*
 * FlashReader.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: james
 */

/* Completed: 3/8/18
 * Part 1: Built an Elaborate SSP Driver. Done.
 * Part 2: Wrote an SPI Flash Reader Application. Done.
 */
#include <demo/lab5_SSP/SSPDriver.hpp>
#include <demo/lab5_SSP/FlashReader.hpp>
/*
 * NEW: 3/9/18
 * Part 3: Write Mutex to guard simultaneous access to the SPI Device.
 *      What is the scenario: Two tasks will be designed to collide
 *      - readSPIFlashID Task: always reads manufacturer/dev ID with
 *          only a single vTaskDelay(1) in between. If signature is
 *          unexpected, it prints fault information.
 *      - readFlashStatRegs Task: always reads the 1 byte from both
 *          status registers with a single vTaskDelay(1).
 *      How do I simulate the problem? Create a problem scenario where
 *      you read unexpected data from the chip signature command
 *      with the assumption the chip signature never changes (it won't)
 *      Then, guard the SPI transaction with a mutex to prove that the
 *      problem is resolved.
 *      - A problem that could a occur is that unexpected data is read
 *      from the chip signature cause both tasks context switch and
 *      metadata shows up in the wrong place of output. Example:
 *      status reg data appears after chip signature command and
 *      vise-versa.*/
// Initialize Mutex
SemaphoreHandle_t spi_bus_lock;

/**
 * Reads information from SPI flash connected to LPC1758 processor via SSP1
 */
void readSPIFlashID(void *params)
{
    SSPDriver *flash = (SSPDriver*)(params); /* SSP1 peripheral */


    /* Local Objects and Variables */
    LabGPIO cs_flash(0, 6); /* P0.6 flash chip select */

    struct adesto_info
    {
        uint8_t manuf_id;
        uint8_t device_id1;
        uint8_t device_id2;
    } adesto_d;

    enum FLASH_CS
    {
        SELECT = 0,
        DESELECT = 1,
    };

    struct Adesto_Opcodes
    {
        const uint8_t rd_id = 0x9F;
    } adesto_op;


    /* Peripheral Initialization */
    cs_flash.setDirection(cs_flash.OUTPUT);

    while(1)
    {
        if( spi_bus_lock != NULL)
        {
            /* Mutex created successfully*/
            // If spi_bus_lock not available immediately, FreeRTOS
            // will sleep task for up to 1sec or will wake task
            // if spi_bus_lock becomes available sooner
            if( xSemaphoreTake(spi_bus_lock, 1000) == pdTRUE )
            {
                /* We're able to obtain mutex, no access to Guarded Resource */

                /* CMD to ask for SPI FLASH Manufacturer ID */
                /* assert cs P0.6 to sel SPI Flash Device */
                cs_flash.set(SELECT);
                /* CMD to ask for SPI FLASH Manufacturer ID */
                flash->transfer(0x9F); //request to read manufacturer id
                adesto_d.manuf_id = flash->transfer(0xFF);

                adesto_d.device_id1 = flash->transfer(0xFF);
                adesto_d.device_id2 = flash->transfer(0xFF);
                /* de-assert cs P0.6 to sel SPI Flash Device */
                cs_flash.set(DESELECT);
                if(adesto_d.manuf_id == 0x1F)
                {
                    u0_dbg_printf("Adesto Manufacturer ID: 0x%02x\n", adesto_d.manuf_id);
                }
                else if(adesto_d.manuf_id != 0x1F)
                {
                    u0_dbg_printf("Oops race condition! Adesto Manufacturer ID doesn't matched expected data: 0x1F\n");
                    vTaskSuspend(NULL);
                }

                if(adesto_d.device_id1 == 0x26)
                {
                    u0_dbg_printf("Adesto Device ID1: 0x%02x\n", adesto_d.device_id1);
                }
                else if(adesto_d.device_id1 != 0x26)
                {
                    u0_dbg_printf("Oops race condition! Adesto Device ID1 doesn't matched expected data: 0x26\n");
                    vTaskSuspend(NULL);
                }

                if(adesto_d.device_id2 == 0x00)
                {
                    u0_dbg_printf("Adesto Device ID2: 0x%02x\n", adesto_d.device_id2);
                }
                else if(adesto_d.device_id2 != 0x00)
                {
                    u0_dbg_printf("Oops race condition! Adesto Device ID2 doesn't matched expected data: 0x00\n");
                    vTaskSuspend(NULL);
                }

                /* We've finished accessing the guarded resource. Release Semaphore */
                xSemaphoreGive(spi_bus_lock);

            }
            else
            {
                u0_dbg_printf("readSPIFlash Task not able to obtain semaphore. Thus, can't access guarded resource.\n");
            }
        }
        vTaskDelay(2000);
    }
}

void readFlashStatRegs(void * params)
{
    SSPDriver *flash = (SSPDriver*)(params); /* SSP1 peripheral */

    //spiQueue = xQueueCreate(1, sizeof(int));

    /* Local Objects and Variables */
    LabGPIO cs_flash(0, 6); /* P0.6 flash chip select */

    enum FLASH_CS
    {
        SELECT = 0,
        DESELECT = 1,
    };

    struct Adesto_Opcodes
    {
        const uint8_t rd_stat = 0xD7;
    } adesto_op;

    //Status Reg bits are stored via Big Endian Figure 25-10
    //Slave outputs Status Register by MSB
    typedef union
    {
        //For the Big Endian Flash
        uint8_t byte;
        struct
        {
            //MSB stored at lowest address
            uint8_t ready_busy_stat: 1; //bit 7
            uint8_t compare_result: 1; // bit 6
            uint8_t density_code: 4; // bits 5-2
            uint8_t protect: 1; // bit 1
            uint8_t page_size: 1; // bit 0
            //LSB
        } __attribute__((packed));
    } at45_stat_r_b1;

    at45_stat_r_b1 stat_b1;

    //Status Reg bits are stored via Big Endian
    //Slave outputs Status Register by MSB
    typedef union
    {
        //For the Big Endian Flash
        uint8_t byte;
        struct
        {
            //MSB stored at lowest address
            uint8_t ready_busy_stat: 1; // bit 7
            uint8_t rsrvd_ftre_use1: 1; // bit 6
            uint8_t erase_prog_err: 1; // bit 5
            uint8_t rsrvd_ftre_use2: 1; // bit 4
            uint8_t sctr_lckdwn_en: 1; // bit 3
            uint8_t prg_supnd_stat_buf2: 1; // bit 2
            uint8_t prg_supnd_stat_buf1: 1; // bit 1
            uint8_t erase_supnd: 1; // bit 0
            //LSB
        } __attribute__((packed));
    } at45_stat_r_b2;

    at45_stat_r_b2 stat_b2;

    /* Peripheral Initialization */
    cs_flash.setDirection(cs_flash.OUTPUT);

    while(1)
    {
        if( spi_bus_lock != NULL )
        {
            /*Mutex created successfully */

            /* See if we can obtain Mutex immediately, else FreeRTOS
             * will sleep task for 1 sec. If mutex becomes available
             * earlier, then FreeRTOS will wake task to take Mutex */
            if( xSemaphoreTake( spi_bus_lock, 1000 ) == pdTRUE )
            {
                /* Able to obtain Mutex and can access guarded resource */

                /* CMD to ask for SPI FLASH Stat Register bytes 1&2 */
                /* assert cs P0.6 to sel SPI Flash Device */
                cs_flash.set(SELECT);

                /* CMD to ask for SPI FLASH Stat Register byte 1&2 */
                flash->transfer(0xD7); //request to read flash stat reg
                stat_b1.byte = flash->transfer(0xFF); // status byte 1
                stat_b2.byte = flash->transfer(0xFF); // status byte 2

                /* de-assert cs P0.6 to sel SPI Flash Device */
                cs_flash.set(DESELECT);

                //Print Status Register Info for byte 1
                u0_dbg_printf("Status Reg Byte 1 Metadata:\n");
                if(stat_b1.page_size)
                {
                    u0_dbg_printf("BIT 0: Device configured for 'power of 2' binary page size (512 bytes)\n");
                }
                else if(!stat_b1.page_size)
                {
                    u0_dbg_printf("BIT 0: Device configured for standard DataFlash page size (528 bytes)\n");
                }

                if(stat_b1.protect)
                {
                    u0_dbg_printf("BIT 1: Sector Protection Enabled\n");
                }
                else if(!stat_b1.protect)
                {
                    u0_dbg_printf("BIT 1: Sector Protection Disabled\n");
                }

                u0_dbg_printf("BITS 5-2: Density Code = %d\n", stat_b1.density_code);

                if(stat_b1.compare_result)
                {
                    u0_dbg_printf("BIT 6: Main memory page data does not match buffer data\n");
                }
                else if(!stat_b1.compare_result)
                {
                    u0_dbg_printf("BIT 6: Main memory page data matches buffer data\n");
                }

                if(stat_b1.ready_busy_stat)
                {
                    u0_dbg_printf("BIT 7: Device is ready\n");
                }
                else if(!stat_b1.ready_busy_stat)
                {
                    u0_dbg_printf("BIT 7: Device is busy with an internal operation\n");
                }

                //Print Status Register Info for byte 2
                u0_dbg_printf("Status Reg Byte 2 Metadata:\n");

                if(stat_b2.erase_supnd)
                {
                    u0_dbg_printf("BIT 0: A sector is erase suspended\n");
                }
                else if(!stat_b2.erase_supnd)
                {
                    u0_dbg_printf("BIT 0: No sectors are erase suspended\n");
                }

                if(stat_b2.prg_supnd_stat_buf1)
                {
                    u0_dbg_printf("BIT 1: A sector is program suspended while using Buffer 1.\n");
                }
                else if(!stat_b2.prg_supnd_stat_buf1)
                {
                    u0_dbg_printf("BIT 1: No program operation has been suspended while using Buffer 1\n");
                }

                if(stat_b2.prg_supnd_stat_buf2)
                {
                    u0_dbg_printf("BIT 2: A sector is program suspended while using Buffer 2\n");
                }
                else if(!stat_b2.prg_supnd_stat_buf2)
                {
                    u0_dbg_printf("BIT 2: No program operation has been suspended while using Buffer 2\n");
                }

                if(stat_b2.sctr_lckdwn_en)
                {
                    u0_dbg_printf("BIT 3: Sector Lockdown command is enabled\n");
                }
                else if(!stat_b2.sctr_lckdwn_en)
                {
                    u0_dbg_printf("BIT 3: Sector Lockdown command is disabled\n");
                }

                if(!stat_b2.rsrvd_ftre_use2)
                {
                    u0_dbg_printf("BIT 4: Reserved for future use\n");
                }


                if(stat_b2.erase_prog_err)
                {
                    u0_dbg_printf("BIT 5: Erase or program error detected\n");
                }
                else if(!stat_b2.erase_prog_err)
                {
                    u0_dbg_printf("BIT 5: Erase or program operation was successful\n");
                }

                if(!stat_b2.rsrvd_ftre_use1)
                {
                    u0_dbg_printf("BIT 6: Reserved for future use\n");
                }

                if(stat_b2.ready_busy_stat)
                {
                    u0_dbg_printf("BIT 7: Device is ready\n");
                }
                else if(!stat_b2.ready_busy_stat)
                {
                    u0_dbg_printf("BIT 7: Device is busy with an internal operation\n");
                }
                /* Finished accessing the guarded resource. Release Mutex */
                xSemaphoreGive( spi_bus_lock );

            }
            else
            {
                u0_dbg_printf("readFlashStatRegs Task not able to obtain Mutex. Thus, can't access guarded resource.\n");
            }
        }
        vTaskDelay(1000);
    }
}

/**
 * Extra Credit:
 * 1. Uses the terminal command to exit this application
 * 2. Reads and Parses MBR Sector from Flash Memory Page 0 (first 512 bytes)
 * Prints the following:
 *    - All meaningful contents of the 4 partition table entries
 *    - Boot Signature
 * Note: Adesto Flash has Page size of 512 Bytes. So get MBR by reading Page 0.
 * To read flash page, use the spi_flash library functions.
 */
void readFlashPage(void *params)
{
    SSPDriver *flash = (SSPDriver*)(params); /* SSP1 peripheral */

    /* By default Flash Page Size is 528 bytes, must configure it to 512 bytes to get MBR*/

    /* Local Objects and Variables */
    LabGPIO cs_flash(0, 6); /* P0.6 flash chip select */

    unsigned char mbr[512];
    //uint8_t PTE[16];

    enum FLASH_CS
    {
        SELECT = 0,
        DESELECT = 1,
    };

    struct Adesto_Opcodes
    {
        const uint8_t rd_page = 0xD2;
    } adesto_op;


    /* Peripheral Initialization */
    cs_flash.setDirection(cs_flash.OUTPUT);

    while(1)
    {
        if( spi_bus_lock != NULL)
        {
            /* Mutex created successfully*/
            // If spi_bus_lock not available immediately, FreeRTOS
            // will sleep task for up to 1sec or will wake task
            // if spi_bus_lock becomes available sooner
            if( xSemaphoreTake(spi_bus_lock, 1000) == pdTRUE )
            {
                /* We're able to obtain mutex, no access to Guarded Resource */

                /* CMD to ask for SPI FLASH Manufacturer ID */
                /* assert cs P0.6 to sel SPI Flash Device */
                cs_flash.set(SELECT);

                //configure device for "power of 2" binary page size(512bytes)
                flash->transfer(0x3D);
                flash->transfer(0x2A);
                flash->transfer(0x80);
                flash->transfer(0xA6);

                //pulse chip select High to allow flash to initiate internally self-timed configuration process
                cs_flash.set(DESELECT);

                vTaskDelay(1000);

                cs_flash.set(SELECT);

                /* CMD to ask for SPI FLASH Manufacturer ID */
                flash->transfer(0xD2); //request to read memory page
                //First 12 bits (A20-A9) at 0b0000_0000_0000_0000, specify to read page 0
                flash->transfer(0x00);//8-bits
                flash->transfer(0x00);//8-bits
                flash->transfer(0x00);//8-bits
                mbr[0] = flash->transfer(0xFF);
                mbr[1] = flash->transfer(0xFF);
                mbr[2] = flash->transfer(0xFF);
                mbr[3] = flash->transfer(0xFF);
                /* de-assert cs P0.6 to sel SPI Flash Device */
                cs_flash.set(DESELECT);


                u0_dbg_printf("Read Page 0 1st Byte: %x", mbr[0]);
                u0_dbg_printf("Read Page 0 2nd Byte: %x", mbr[1]);
                u0_dbg_printf("Read Page 0 3rd Byte: %x", mbr[2]);
                u0_dbg_printf("Read Page 0 4th Byte: %x", mbr[3]);

                vTaskDelay(1);

                /* We've finished accessing the guarded resource. Release Semaphore */
                xSemaphoreGive(spi_bus_lock);
            }
            else
            {
                u0_dbg_printf("readSPIFlash Task not able to obtain semaphore. Thus, can't access guarded resource.\n");
            }
        }
    }
}

void runFlashReader(void)
{
    SSPDriver ssp;
    SSPDriver *flash;

    flash = &ssp;

    /* This type of semaphore uses priority inheritance, so a task 'taking'
     * a semaphore MUST ALWAYS 'give' the semaphore back once the semaphore
     * is no longer required.*/
    spi_bus_lock = xSemaphoreCreateMutex();

    /* Define Constants */
    const uint8_t data_transfer_size = ssp.TRANSFER_8_BITS; /* Controls # of bits transferred in each frame*/
    const uint8_t prescaler_divider = 8; /* Even value by which SSP_PCLK is divided to yield prescaler clk */

    //Prescaler CLK Out = PCLK/( CPSR * (SCR+1) ) = 96/8 = 12MHz
    ssp.init(ssp.SSP1, data_transfer_size, ssp.SPI, prescaler_divider);

    xTaskCreate(readSPIFlashID, "readFlashID", 2048, (void *)flash, 1, NULL);
    xTaskCreate(readFlashStatRegs, "readFlashStatRegs", 2048, (void *)flash, 1, NULL);
    //xTaskCreate(readFlashPage, "readFlashPage", 2048, (void *)flash, 1, NULL);
    vTaskStartScheduler();
}

