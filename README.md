## CMPE 146 Lab Projects

## Directions to Run Projects

1\. Download the repo

2\. Open the project in Eclipse IDE (or favorite IDE)

3\. Head to `lpc1758_firmware_labs/L5_Application/` path to open `main.cpp` in Eclipse

4\. Out of the enumeration list, choose the lab you want to run. For example **L7_MMA_Tracker**

5\. Build the project (ctrl+B)

6\. Upload firmware onto your ARM microcontroller (SJ One Board)

7\. View the display output in a telemetry system (Ex: Hercules)

## Semester Short-term Lab Projects:

### Lab 1 Multiple Tasks: Preemptive Context Switch Analysis

  - Summary: In this lab, two tasks were created for the scenarios in which both had the same priority and then different priorities. For this lab, I learned about **FreeRTOS Scheduler** and **OS Tick Frequency** and **OS Tick Interrupt**. When the firmware application runs, the FreeRTOS scheduler is called by the OS Tick Interrupt based on an OS Tick Frequency. The scheduler will context switch between both tasks based on their priority level and whether or not they are sleeping. Example: Same priority tasks share the CPU time, so the scheduler will **context switch** between them with a fequency of 1ms while different priority tasks only the scheduler will context switch from high to low priority task when the high priority task sleeps.
  
  - Source Code: 
  
[lab1_Multi_Tasks](./L5_Application/demo/lab1_Multi_Tasks/)

  - Source Code Path:

~~~
lpc1758_firmware_labs/L5_Application/demo/lab1_Multi_Tasks/
~~~

### Lab2 GPIO: Blink LED 

  - Summary: A **GPIO Driver Class** was created based on the UM10360 datasheet, which is implemented in LabGPIO.hpp and LabGPIO.cpp. This driver class was utilized to build a **Blink LED FreeRTOS Application**. One task **vReadSwitch** was created to read if a switch was pressed and upon being pressed, the task would set a global flag that the other task **vControlLED** would check to control whether **vControlLED** illuminated the LED.
  
  - Source Code:

[lab2_Gpio](./L5_Application/demo/lab2_Gpio)

  - Source Code Path:
  
~~~
lpc1758_firmware_labs/L5_Application/demo/lab2_Gpio/
~~~

### Lab3 EINT: Interrupt Driven Blink LED

  - Summary: The purpose of the External Interrupts Lab was to add on external interrupt capability to the GPIO Port 0 and Port 2 pins. The **LabGPIOInterrupts Class** contains data members for storing GPIO Port 0 and 2 Rising and Falling Edge interrupt functions. It also contains an "attachInterruptHandler" method to add a function to an appropriate GPIO Port, so that function will run when an external interrupt on a pin takes place. The class also contains a "handle_interrupt" method to call on the function that needs to run when an external interrupt occurs. In the **FreeRTOS application** space, with the addition of external interrupt capability, I was able to remove the **vReadSwitch** task from the last lab because the **interrupt service routine (ISR)** gets run everytime a switch is pressed, which calls the appropriate function to perform the action in correspondence to that interrupt. In our case, when an external interrupt is edge triggered by the switch, the ISR gives up the binary semaphore and the appropriate task, **vIntControlLEDRise,** **vIntControlLEDFall** or **vIntControlLEDBoth**, waiting to take the semaphore wakes up to blink the LED.
  
  - Source Code:
  
[lab3_Eint](./L5_Application/demo/lab3_Eint)

  - Source Code Path:

~~~
lpc1758_firmware_labs/L5_Application/demo/lab3_Eint/
~~~

### Lab4 ADC + PWM: Illuminate RGB Via Potentiometer

  - Summary: The purpose of this lab was to build a **FreeRTOS application** that reads a potentiometer analog signal using an ADC pin, outputs the converted digital signal onto a PWM pin wire to control the color illumination of an RGB LED. This user application has two FreeRTOS tasks that are connected by a FreeRTOS queue with **vReadPotentiometer** task instantiating an object of the **ADC Driver Class** to ingest potentiometer data and **vDriveRGBLED** task instantiating an object of the **PWM Driver Class** to control the color of the RGB LED. Before the user runs the application, they can go to the user code and pass into the "vDriveRGBLED" task parameter, the color they want the RGB to be from RED, GREEN, BLUE, PURPLE, etc. Once the code is cross compiled to the SJOne Board, the user is able to control the illumination of the RGB LED.

  - Source Code:
  
[lab4_Adc_Pwm](./L5_Application/demo/lab4_Adc_Pwm)
  
  - Source Code Path:
  
~~~
lpc1758_firmware_labs/L5_Application/demo/lab4_Adc_Pwm/
~~~

### Lab5 SSP: Flash Reader

  - Summary: The purpose of this lab was to build a **SSP Driver Class** that can be used for a **FreeRTOS Application** to send data bytes to the SPI Flash to read its manufacturer ID, device ID and status registers. There are two tasks, **readSPIFlashID** and **readFlashStatRegs** in which both try to take the **FreeRTOS mutex semaphore**, only one is able to take it and run. Once that task is done performing it operations, it gives up the semaphore and the other task that was blocked, wakes up to take the semaphore. 
  
  - Source Code:
  
[lab5_SSP](./L5_Application/demo/lab5_SSP)
  
  - Source Code Path:
  
~~~
lpc1758_firmware_labs/L5_Application/demo/lab5_SSP/
~~~

### Lab6 UART: User Calculator

  - Summary: The purpose of this lab was to build a **UART Driver Class** that can be leveraged to build a **FreeRTOS calculator application** in which one microcontroller acts as the arithmetic logic unit or calculator and the other microcontroller acts as the user. **vUser** task runs on the user microcontroller while **vALU** task runs on the calculator microcontroller and both boards are interfaced by UART TX and RX lines. One TX to RX wire is used to allow the user to tranmsmit their arithmetic expression (operand1, operator, operand2) to the calculator and the other RX to TX wire allows the calculator to send the computed result (one digit or two digits and accounts for negative numbers) back to the user. On each wire, one char byte is transmitted at a time. Once the user receives the result, the value will be properly displayed in the hercules telemetry system while accounting for the order in which the char bytes were received.
  
  - Source Code:
  
[lab6_UART](./L5_Application/demo/lab6_UART)
  
  - Source Code Path:
  
~~~
lpc1758_firmware_labs/L5_Application/demo/lab6_UART/
~~~

### Lab7 MMA: Micromachined Accelerometer Tracker

  - Summary: The purpose of this lab was to create a **producer** and **consumer** task that combine to rotationally **track the SJOne Board's orientation**, whether it is facing **UP, DOWN, LEFT or RIGHT**. In the producer task, the professor's accelerometer sensor object is utilized to get the **X,** **Y** and **Z** rotational coordinates of the boar's orientation. This information is sent over FreeRTOS queue to the consumer task to display on hercules whether the board is UP, DOWN, LEFT or RIGHT.
  
  - Source Code:

[lab7_MMA_Tracking](./L5_Application/demo/lab7_MMA_Tracking)
  
  - Source Code Path:
  
~~~
lpc1758_firmware_labs/L5_Application/demo/lab7_MMA_Tracking/
~~~

### Lab8 I2C: I2C Slave Register Alert Monitor

  - Summary: For this lab, we were given the professor's **pre-existing I2C_Base Master Driver Class**. Our objective for this lab was to **extend** the master's transaction based state machine to be able to perform slave transmit and receive transactions. I added an **initSlave** method to the **Driver Class** and the following **slave receiver states** to the **I2C State Machine**: **writeModeAckedBySlave, writeDataAckedBySlave and writeLastByteAckedBySlave**. I also added the **slave transmitter states**: **readModeAckedBySlave, writeDataAckedByMaster** and **writeLastByteAckedByMaster**. Therefore, now the user can use the I2C Driver for slave devices too. In the **user application**, an I2C object is instantiated from the I2C2 child class of the I2C_Base Base class. The I2C slave device is initialized and then there is a constant check taking place to **verify if any I2C Slave Device registers have changed**. **If a change occurred**, then a string is output to the **display which register is now different**. In the class demo, there is a I2C master board and an I2C slave board. From the master board the user can override the slave's registers using the hercules terminal's I2C command. Once a register's value is changed, data is transmitted from master to slave on the SDA line while SCL controls the speed of the data transmission.
  
  - Driver Source Code:
  
[I2C Driver Code](./L2_Drivers/base)
  
  - Driver Source Code Path:

~~~
lpc1758_firmware_labs/L2_Drivers/base/
~~~

  - User Application Source Code:

[lab8_I2C_Slave_Device](./L5_Application/demo/lab8_I2C_Slave_Device)
  
  - User Application Source Code Path:
  
~~~
lpc1758_firmware_labs/L5_Application/demo/lab8_I2C_Slave_Device/
~~~

### Lab9 FreeRTOS Watchodgs

  - Summary: In this lab, three tasks were created: **getLightData**, **pullLightData** and **watchdogMonitor**. While both **getLightData** and **pullLightData** sensor communicate via a FreeRTOS queue, a **watchdogMonitor** task is constantly monitoring the health of the two tasks by verifying they set their appropriate bits in the 32 bit eventGroup. If the **getLightData** task never sets it's bit, then the **watchdogMonitor** task will log a WARN message that the task never set it's bit. Similar action will take place in the event that **pullLightData** doesn't set it's bit in the eventGroup. If both tasks are down, the **watchdogMonitor** will log that both tasks are not responding. All these logs are being saved in a file called stuck.text on the SJ One Board's Flash memory. If the **pullLightData** task is retrieving data, it will save the light percentage sensor data into a "sensor.txt" file. Every 60 seconds, the **watchdogMonitor** saves the CPU usage information into a "cpu.txt" file.
  
  - Source Code:
  
[lab9_Watchdogs](./L5_Application/demo/lab9_Watchdogs)
  
  - Source Code Path:
  
~~~
lpc1758_firmware_labs/L5_Application/demo/lab9_Watchdogs/
~~~

## Semester Long-term MP3 Player Project:

The MP3 Player project consists of four components: SJ One Board (powered by LPC1758), MP3 Files on SD Card, Nokia LCD display and MP3 Decoder that combine to create an music player device. 

For the project, I designed and implemented two frameworks in C/C++: SPI LCD Display and SPI MP3 Decoder. My team utilized my frameworks to integrate them with their components to finish the application. Our demo was able to play digital audio files, select their desired song from a playlist and adjust volume.

### Nokia 5110 SPI LCD Display

  - Summary: The LCD display is an 84 * 48 black and white pixel display similar to the nokia cell phone displays. I leveraged PCD8544 LCD controller/driver datasheet to build the C code framework for this LCD display. Once I was able to establish communication between the SJ One Board and the display, such as running the simple graphic demo from the display, I moved to referencing the higher level arduino code framework to see how they generated all different kinds of ascii characters on the display. Also my objective was to see how they controlled which location (X, Y coordinate) the character appeared on the display. Once I reviewed and understood their approach, I adapted the code from the arduino (ATMEL) framework to my own driver framework on the LPC1758. I took advantage of their ascii table two-dimensional array and one-dimensional display map array as well as adapting some of their functions for my own needs. Thus, now I am able to generate single ascii characters on the display and even output strings on multiple lines on the display. You will probably notice the code is very long and many functions have alot of parameters, the reason is because I needed to get something working fast for the MVP date of May 23. If I have more time, I plan to transition my C framework to a C++ class for reduce code reuse and reduce function parameters and take advantage of object oriented programming.
  
  - Source Code:
  
[MP3 Player Display Code](./L5_Application/mp3_player/mp3SubApplications/display)
  
  - Source Code Path:
  
~~~
lpc1758_firmware_labs/L5_Application/mp3_player/mp3SubApplications/display/
~~~

### VS1053b SPI MP3 Decoder

  - Summary: The MP3 Decoder is the core of the project and is a VS1053b digital signal processing chip that takes digital audio data and converts it to analog signal that we can listen to through our headphones. To create the foundation of the framework, I leveraged the VS1053b datasheet. I analyzed the timing diagrams to implement the functions for writing and reading opcodes from/to serial control interface (sci) registers. I wrote a function for writing data over the serial data interface, which is directly processed by the MP3 decoder. Other functions that are implemented based on these three fundamental functions are hardReset, sineTest, softReset and playWholeFile. For the 4 functions mentioned previously, I leveraged the datasheet's english pseudocode explanation. Currently if the user wants to test the audio output of the MP3 Decoder, they can run the vTestMP3Decoder task. The second task, which hasn't been tested yet, vPlayMP3Music plays an entire audio file in which the user could listen to the song through the headphones. Currently, I am working with my team to finish implementing the other features of the decoder.
  
  - Source Code:
  
[MP3 Audio Decoder Code](./L5_Application/mp3_player/mp3SubApplications/audioDecoder)
  
  - Source Code Path:
  
~~~
lpc1758_firmware_labs/L5_Application/mp3_player/mp3SubApplications/audioDecoder/
~~~

### MP3 Player Gameboy Demo

  - Summary: MP3 Player Embedded Application consists of a LCD Display module written using SPI protocol, Digital to Audio Decoder module written using SPI protocol, SD Card module written using SPI protocol and a Playback controls module written using GPIO protocol. The data pipeline involves mp3 files being read from the SD Card using the SD Card module, then transmitted into the Digital to Audio Decoder to be heard from headphones connected to the SJOne board's audio jack. The user can select songs displayed on the LCD Display, which is sent from the SD Card module to the LCD Display module.

  - Source Code:
  
  [MP3 Player Code](./MP3Player)
