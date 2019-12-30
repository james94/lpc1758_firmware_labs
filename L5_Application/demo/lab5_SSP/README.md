# SPI Flash Reader Project

This project is part of SJSU's Real-Time Embedded Systems Co-Design

## Summary

## Deliverables Achieved for Project

- Implemented SSP Driver Class to support SSP0 and SSP1 peripheral controllers

- Utilized GPIO Driver Class for SPI device chip select signals

- Tested communication to SPI Adesto Flash and implementing code to print out metadata of each required register, success

- Checked if SSP1 master receives appropriate Manfucturer ID (8-bit) metadata from Flash, print hex value, success

- Checked if SSP1 master receives appropriate Device ID1 (8-bit) and Device ID2 (8-bit) metadata from Flash, print hex values, success

- Checked if SSP1 master receives appropriate Status Register (16-bit) metadata from Flash 

- Worked on creating/using bit field structure map for Status Register, so I can print the bit information

- Implemented mutex semaphore to solve an event in which a race condition occurs: example, when two slave devices try to access the same ssp peripheral, use a mutex, so devices don't blow up

- Used Logic Analyzer to capture screenshots of the SPI decoded waveform retrieving the manufacturer ID
 