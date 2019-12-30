/*
 * UARTDriver.cpp
 *
 *  Created on: Mar 13, 2018
 *      Author: james
 */

#include <demo/lab6_UART/UARTDriver.hpp>

constexpr LPC_UART_TypeDef * UARTDriver::UART[];

//Learn diagram first (UM 10360: Fig 48)
//To initialize a const member variable, must do it in an initialization list
UARTDriver::UARTDriver(uint32_t uart_pc, bool uart_int) : m_uart_pc(uart_pc)
{
    m_uart_int = uart_int;
    rx_fifo = xQueueCreate(1, sizeof(char));
}

bool UARTDriver::init(uint32_t baud_rate, WordLengthSelect item_size)
{
    u0_dbg_printf("beginning of init: UART - %i\n", (m_uart_pc + 2));

    bool init_status = true;
    if(m_uart_pc == UART2)
    {
        //Init UART-2 First for testing
        //P2.8 Tx
        //P2.9 Rx
        //1. Power: PCONP reg set PCUART2 bit (Table 46)
        LPC_SC->PCONP |= (1 << 24); //Set PCUART2 Bit24
        //2. Peripheral Clock: PCLKSEL1 reg set PCLK_UART2 bit (Table 40)
        LPC_SC->PCLKSEL1 &= ~(3 << 16); //Clear PCLK_UART3 Bit 19:18
        LPC_SC->PCLKSEL1 |= (1 << 16); //Set PCLK_UART2 Bit17:16

        //3. Set Baud Rate: U2LCR reg set DLAB=1
        UART[m_uart_pc]->LCR |= (1 << 7); //Sets DLAB=1 Enable access to Divisor Latches DLL, DLM

        // Calculate Baud rate using DLL and DLM
        // Baud Rate = (PCLK) / ( (16 * (( 256 * DLM ) + DLL) ) * (1 + (DivAddVal/MulVal) ) )
        // DLL = ( (PCLK) / (baud_rate * 16) ) - (DLM * 256)
        // DLL = ( (PCLK) / (baud_rate << 4) ) - (DLM << 8)
        UART[m_uart_pc]->DLM &= ~(0xFF << 0); //7:0 DLM Value
        //
        UART[m_uart_pc]->DLL = ( sys_get_cpu_clock() / (baud_rate << 4) ) - (UART[m_uart_pc]->DLM << 8);

        // Optional: Set fractional rate divider for great versatility

        //4. Set Word Length Select (WLS): U0/2/3LCR reg set WLS to 3
        UART[m_uart_pc]->LCR &= ~(3 << 0);
        UART[m_uart_pc]->LCR |= (item_size << 0); //Set WLS 8 bit char length

        //5. Enable UART FIFO for proper UART operation: U0/2/3FCR set FIFO Enable Bit (BIT 0) ... 16 byte Rx/Tx FIFOs
        UART[m_uart_pc]->FCR |= (1 << 0); //Active High Enable both UART Rx/Tx FIFO and enable access to FCR[7:1] bits

        //6. Pins: PINSEL select UART2 Pins
        LPC_PINCON->PINSEL4 &= ~(3 << 16);
        LPC_PINCON->PINSEL4 |= (2 << 16); //P2.8 UART-2 Tx

        LPC_PINCON->PINSEL4 &= ~(3 << 18);
        LPC_PINCON->PINSEL4 |= (2 << 18); //P2.9 UART-2 Rx

        UART[m_uart_pc]->LCR &= ~(1 << 7); // Set Divisor Latch Access Bit (DLAB) to 0 to access THR, RBR always write only, also needed for Interrupts

        //Check if user wants UART Interrupts enabled
        if(m_uart_int == INT_ENABLED)
        {
            //Q#1: What causes the RDA INT to activate?
            //7. The UARTn Rx FIFO reaches trigger level defined by UnFCR[7:6]
            //The Rx Trigger Level is 0, means when 1 character is written, then the interrupt is activated
            UART[m_uart_pc]->FCR &= ~(3 << 6);
            //8. Enable Interrupt in NVIC using appropriate enum
            // Configure NVIC to notice UART2 IRQs
            NVIC_EnableIRQ(UART2_IRQn);
        }
    }
    else if(m_uart_pc == UART3)
    {
        //Init UART-3
        //P0.0 Tx
        //P0.1 Rx
        //1. Power: PCONP reg set PCUART3 bit (Table 46)
        LPC_SC->PCONP |= (1 << 25);
        //2. Peripheral Clock: PCLKSEL1 reg set PCLK_UART3 bit (Table 40)
        LPC_SC->PCLKSEL1 &= ~(3 << 18);
        LPC_SC->PCLKSEL1 |= (1 << 18);
        //3. Set Baud Rate: U3LCR reg set DLAB=1
        UART[m_uart_pc]->LCR |= (1 << 7); // set DLAB = 1, enable access to DLL/DLM
        // Baud Rate = (PCLK) / ( (16 * (( 256 * DLM ) + DLL) ) * (1 + (DivAddVal/MulVal) ) )        // Baud Rate = (PCLK) / ( (16 * 256 * (DLM + DLL) ) * (1 + (DivAddVal/MulVal) ) )
        // DLL = ( (PCLK) / (baud_rate * 16) ) - (DLM * 256)
        UART[m_uart_pc]->DLM &= ~(0xFF << 0);
        UART[m_uart_pc]->DLL = ( sys_get_cpu_clock() / (baud_rate * 16) ) - LPC_UART3->DLM * 256;
        //4. Set Word Length Select (WLS): U3LCR reg set WLS to 3
        UART[m_uart_pc]->LCR |= (item_size << 0);
        //5. Enable UART FIFO for proper UART operation: U3FCR set FIFO Enable Bit (BIT 0) ... 16 byte Rx/Tx FIFOs
        UART[m_uart_pc]->FCR |= (1 << 0);
        //6. Pins: PINSEL select UART3 Pins
        LPC_PINCON->PINSEL0 &= ~(3 << 0); //Clear P0.0 function to GPIO
        LPC_PINCON->PINSEL0 |= (2 << 0); //Select P0.0 UART-3 Tx

        LPC_PINCON->PINSEL0 &= ~(3 << 2); //Clear P0.1 function to GPIO
        LPC_PINCON->PINSEL0 |= (2 << 2); //Select P0.1 UART-3 Rx

        UART[m_uart_pc]->LCR &= ~(1 << 7); // set DLAB = 0, enable access to THR, RBR, also needed for Interrupts
        //Check if user wants UART Interrupts enabled
        if(m_uart_int == INT_ENABLED)
        {
            //Q#1: What causes the RDA INT to activate?
            //7. The UARTn Rx FIFO reaches trigger level defined by UnFCR[7:6]
            //The Rx Trigger Level is 0, means when 1 character is written, then the interrupt is activated
            UART[m_uart_pc]->FCR &= ~(3 << 6);
            //8. Enable Interrupt in NVIC using appropriate enum
            // Configure NVIC to notice UART2 IRQs
            NVIC_EnableIRQ(UART3_IRQn);
        }
    }

    u0_dbg_printf("end of init: UART - %i\n", (m_uart_pc + 2));

    return init_status;
}

// @param item_size The size, in bits, required to hold each item in the queue
void UARTDriver::transfer(char send)
{
    //1. Set Divisor Latch Access Bit (DLAB) in LCR to zero to access THR, always write only
    //Do a bit check for LCR
    //2. Line Status Register (LSR): Contains flags for transmit status, including line errors
    while(!( UART[m_uart_pc]->LSR & (1 << 5) )); //loop while not empty, break when THR is empty
    //3. Transmit Holding Reg (THR): Next Char to be transmitted written here
    //7:0 THR is the newest char in Tx FIFO, byte will be sent when it reaches bottom of FIFO and Tx is available
    UART[m_uart_pc]->THR = (send << 0);

    u0_dbg_printf("UART - %i Tx: %c\n", (m_uart_pc + 2), send);
}

char UARTDriver::receive()
{
    char byte = 'x';

    //1. Verify Divisor Latch Access Bit (DLAB) in LCR set to zero to access RBR, always read only
    //2. Read Line Status Register (LSR): Contains flags for receive status, including line errors
    while(!( UART[m_uart_pc]->LSR & (1 << 0) )); //loop while empty, break when RBR not empty
    //3. Read byte from Receive Buffer Reg (RBR): Contains next received char to be read
    //7:0 RBR contains oldest received byte in UARTn Rx FIFO
    //LSB represents oldest received data bit, if char received < 8 bits, unused MSB zero padded
    byte = UART[m_uart_pc]->RBR;

    u0_dbg_printf("UART - %i Rx: %c\n", (m_uart_pc + 2), byte);

    return byte;
}

void UARTDriver::intReceive()
{
    u0_dbg_printf("INT receive: UART - %i\n", (m_uart_pc + 2));

    char byte = 'x';
    //1. Verify Divisor Latch Access Bit (DLAB) in LCR set to zero to access RBR, always read only
    //2. Read Line Status Register (LSR): Contains flags for receive status, including line errors
    while(!( UART[m_uart_pc]->LSR & (1 << 0) )); //loop while empty, break when RBR not empty

    //3. Read byte from Receive Buffer Reg (RBR): Contains next received char to be read
    //7:0 RBR contains oldest received byte in UARTn Rx FIFO
    //LSB represents oldest received data bit, if char received < 8 bits, unused MSB zero padded
    byte = UART[m_uart_pc]->RBR;

    if(rx_fifo != NULL)
    {
        if(xQueueSend(rx_fifo, &byte, 500) != pdTRUE)
        {
            u0_dbg_printf("Failed to send data after 500ms\n");
        }
        else
        {
            u0_dbg_printf("Placed UART Rx Data onto queue\n");
        }
    }

}

//Checks for the type of UART interrupt the user wants enabled for UARTn channel
//Then enables that particular UART interrupt
//Places the function that will be executed during the interrupt into the function pointer
//Sets the interrupt_status flag to true when successfully enabling an UARTn channel interrupt
//Returns whether setting the interrupt was successful
bool UARTDriver::attachUARTInterruptHandler(uint8_t port, uint8_t pin, void (*pin_isr)(void), UARTInterruptCondition_E condition)
{
    bool int_status = false;

    if(condition == RDAInterrupt)
    {
        UART[m_uart_pc]->IER |= (1 << 0); //Enable RBR Interrupts
        UARTn_Int_RDA[m_uart_pc] = pin_isr;
        int_status = true;
        u0_dbg_printf("Attached RBA Int to UART-%i on P%i P%i\n", (m_uart_pc + 2), port, pin);
    }
    else
    {
        u0_dbg_printf("Invalid UARTn interrupt condition selected\n");
        int_status = false;
    }
    return int_status;
}

/*
* This function is invoked by the CPU (through c_uartn_handler) asynchronously when a Port/Pin
* interrupt occurs. This function is where you will check the UARTn status, such as IO0IntStatF,
* and then invoke the user's registered callback and find the entry in your lookup table.
*/
void UARTDriver::handle_interrupt(void)
{
    //When UnIIR[0] is 0, a non auto-baud interrupt is pending and the cause of
    //the interrupt can be determined by evaluating UnIIR[3:1]
    if( !(UART[m_uart_pc]->IIR & (1 << 0)) )
    {
        u0_dbg_printf("At least one UART-%i interrupt is pending\n", (m_uart_pc + 2));

        //The UnIIR must be read to clear the interrupt prior to exiting the Interrupt Service Routine

        //#1: UARTn RLS Interrupt (UnIIR[3:1] == 011) is highest priority interrupt and is set
        //whenever any one of four error conditions occur on UARTn Rx input:
        //1. Overrun Error (OE)
        //2. Parity Error (PE)
        //3. Framing Error (FE)
        //4. Break Interrupt (BI)
        //NOTE: UARTn Rx Error Condition which set the interrupt can be observed via UnLSR[4:1]
        //NOTE: the interrupt is cleared upon UnLSR READ

        //#2: UARTn RDA Interrupt (UnIIR[3:1] == 010) shares 2nd priority level
        //with CTI interrupt (UnIIR[3:1] == 110)
        //Q#1: What causes the RDA INT to activate?
        // --- The UARTn Rx FIFO reaches trigger level defined by UnFCR[7:6] or Rx data available
        //Q#2: What causes the RDA INT to reset?
        // --- The UARTn Rx FIFO depth falls below trigger level
        //Q#3: What happens when the RDA INT goes active?
        // --- The CPU can read a block of data defined by trigger level


        //Determine if the type of interrupt pending is UART RDA Interrupt IIR[3:1]
        if(UART[m_uart_pc]->IIR & (2 << 1))
        {
            u0_dbg_printf("Receive Data available interrupt is pending\n");

            if(UARTn_Int_RDA[m_uart_pc] != NULL)
            {
                UARTn_Int_RDA[m_uart_pc]();
                u0_dbg_printf("Called Callback: UART_RDA-%i\n", (m_uart_pc + 2));

            }
        }

        //#3: UARTn CTI Interrupt (UnIIR[3:1] == 110) is 2nd priority level interrupt
        //Q#1: What causes the CTI INT to set?
        // --- The UARTn Rx FIFO contains at least one character, but no activity has occurred in 3.5 to 4.5 character times
        //Q#2: What causes the CTI INT to clear?
        // --- Any UARTn Rx FIFO activity(read or write of UARTn RSR)
        //Q#3: What is the "intent" of the CTI INT?
        // --- CTI INT flushes the UARTn RBR after a message has been received that's not a multiple of trigger level size
        //Example, if a peripheral wished to send a 105 character message
        //and trigger level was 10 characters, the CPU would receive 10 RDA interrupts resulting
        //in the transfer of 100 characters and 1 to 5 CTI interrupts (depending on service routine)
        //resulting in the transfer of the remaining 5 characters
    }

}

UARTDriver::~UARTDriver()
{
    // TODO Auto-generated destructor stub
}
