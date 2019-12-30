/*
 * FlashReader.hpp
 *
 *  Created on: Mar 8, 2018
 *      Author: james
 */

#ifndef FLASHREADER_HPP_
#define FLASHREADER_HPP_

#include "tasks.hpp"
#include "semphr.h"
#include <demo/lab2_Gpio/LabGPIO.hpp>

/**
 * Reads and prints information from SPI flash connected to LPC1758 processor
 */
void readSPIFlash(void *params);

/**
 * Reads and prints in human readable format metadata from both of the
 * SPI Flash's 1 Byte Status Regs
 */
void readFlashStatRegs(void * params);
/* Creates two tasks: one reads flash chip signatures, other reads flash status registers*/
void runFlashReader(void);

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

/**
 * Read Main Memory Page 0, how?
 * - To start a page read using binary page size (512 bytes),
 * the opcode 0xD2 must be clocked
 * into device followed by 3 address bytes and 4 dummy bytes
 * - The first 12 bits (A20-A9) of 21-bit address sequence specify
 * which page of the main memory array to read
 * - The last 9 bits (A8-A0_ of the 21-bit address sequence specify
 * the starting byte address within that page.
 * - The dummy bytes that follow the address bytes are sent to
 * initialize the read operation.
 * - Following the dummy bytes, additional pulses on SCK result in data being output on SO pin.
 *
 * Note: CS_ pin must remain low during loading of opcode, address bytes, dummy bytes and
 * the reading of data.
 *
 * Unlike Continuous Array Read command, when the end of a page in main memory is reached,
 * the device will continue reading back at the beginning of the same page rather
 * than the beginning of the next page.
 *
 * A low-to-high transition on CS_ pin will terminate the read operation on tri-state
 * output pin (SO)
 *
 * Opcode: 0xD2
 * Page 0 Address Byte:
 * Dummy Bytes: 0xFF
 */

/* Class to parse Adesto AT45 Flash Data*/
class FlashFatFs
{
    /*FAT Information
     * - File System includes data structures used to manage all files in storage device
     * - File System uses File Allocation Table (FAT) data structure to organize and store info about file locations in a partition
     * - A Partition stores all metadata about files and file systems in its first sector 'Partition Boot Sector'
     * - Each Partition in storage disk has its own Boot Sector, which also holds boot code if partition is bootable
     * - Info about all such partitions is located in Partition Table, which is in 1st sector of disk 'Master Boot Record'*/

    /*Master Boot Record
     * - is the first 512 bytes (Page 0 of memory) of flash
     * - contains pointers to where the partitions are placed in the storage device
     * 3 Parts:
     * - the first 446 bytes is bootstrap code area
     * - after that is the partition table with four entries
     *      - each entry is 16 bytes long
     * - then the boot signature (0x55AA) */

    /*A partition table describes partitions of a storage device
     * The boot sector alias is partition sector
     *Bootstrap code are instructions that identify the configured bootable partition, that load and execute volume boot record (VBR) as chain loader */

    /*Where to see a structure of the Master Boot Record: Sector Layout?
     *
     * Where to find the structure of the partition entry 'Master Boot Record: Partition Table Entries'? */
private:
    /* Get the information about FAT File System such as Sector Size, total sectors etc, parse
     * MBR partition table to get the page number of FATFS Boot Sector*/
    uint32_t fat_boot_sector;
public:
    /* Constructor */
    FlashFatFs();

    /* Creates and uses structure mapping for whole 512 byte page */
    typedef union /* Partition Table Entry */
    {
        uint8_t PTE[16];
        struct
        {
            /* Map all 16 bytes of PTE */
            uint8_t partition_state;
            uint8_t partition_head_start;
            uint16_t partition_sector_start;
            uint8_t partition_type;
            uint8_t partition_head_end;
            uint16_t partition_sector_end;
            uint32_t num_sctrs_btwn_mbr_1st_sctr;
            uint32_t num_sectors_in_partition;
        } PTE_fields;
    }__attribute__((__packed__))PTE_t;

    /* TODO: Defines struct FatBootSector */

    typedef union
    {
        unsigned char mbr[512];
        struct{
            unsigned char boot_code[446];
            PTE_t PTE1;
            PTE_t PTE2;
            PTE_t PTE3;
            PTE_t PTE4;
            uint16_t boot_signature; // 0x55AA
        }mbr_fields;
    } __attribute__((__packed__)) MBR;

    /* How to get FAT File System Information?
     * 1. Read Master Boot Record to obtain location of starting sector of FATFS partition, which is its boot sector.
     * 2. Read the Boot Sector of the partition to obtain the FATFS information*/


    void flashReadSectors(unsigned char mbr[512], uint8_t page_num, uint8_t count);


};

#endif /* FLASHREADER_HPP_ */
