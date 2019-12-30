/*
 * UARTCalcApp.cpp
 *
 *  Created on: Mar 15, 2018
 *      Author: james
 */

#include <demo/lab6_UART/UARTCalcApp.hpp>

//Create UARTDriver object, Set UART Channel
//vUser
UARTDriver uart(uart.UART3, uart.INT_DISABLED);
//vALU
//UARTDriver uart(uart.UART2, uart.INT_DISABLED);

/*Size of Items sent*/
const uint32_t MAX_ELEMENTS = 3;

void c_uart3_handler(void)
{
    uart.handle_interrupt();
}

void c_callbackIntReceive(void)
{
    uart.intReceive();
}

//points to the address of the array, then prints contents
//by pointing to each element and outputting it to the console with leading zeros
//Reference: https://stackoverflow.com/questions/6814533/using-pointers-to-display-content-of-array
void printCharArr(const char *c_result, const uint32_t MAX_ITEMS)
{
    for(int i = (MAX_ITEMS-1); i >= 0; --i)
    {
        u0_dbg_printf("%c", *(c_result + i)); //c_result[i]
    }
    u0_dbg_printf("\n");
}

//Prints the mathematical computation that was performed and
//Prints char array integer result without leading zeros
void printCharArrResult(const char *c_result, const char operand1, const char operand2, const char operation, const uint32_t MAX_ITEMS)
{
    int32_t j = 0;

    u0_dbg_printf("Operation: %c %c %c = ", operand1, operation, operand2);

    //Finds first non-0 byte, starting at the highest digit
    //Reference: https://stackoverflow.com/questions/24839107/how-to-ignore-remove-leading-zeros
    for(j = (MAX_ITEMS-1); j >= 0; --j)
    {
        //'0' does not equal '0' is false, so doesn't go into if condition
        //go into if condition if any number or '-'
        if(c_result[j] != '0')
        {
            break;
        }

    }
    //Prints all digits
    while(j >= 0)
    {
        u0_dbg_printf("%c", c_result[j]);
        --j;
    }
    u0_dbg_printf("\n");
}

//grab each digit of the integer i_result, convert it to char, use pointer
//to point to each index of the char array and store the digit in the appropriate location
//break if no more digits to store or if pointer to the null terminator
//null terminator means we have reached a spot out of bounds of the array
//Function converts integer digits to char ascii equivalent, stores into array
//Reference: https://stackoverflow.com/questions/5378768/returning-arrays-pointers-from-a-function
void wIntToCharArr(int32_t i_result, char *c_result, uint32_t arr_size)
{
    int32_t digit = 0;
    int32_t int_to_char = 0;
    uint32_t c = 0;
    int32_t temp = i_result; //12
    bool negative = false;
    //store each digit from result in char array, break if pass last digit, break if exceed array indexes
    while(temp != 0 && *c_result != '\0')
    {
        u0_dbg_printf("temp = %i\n", temp);
        u0_dbg_printf("c = %i\n", c);

        //check if negative number
        if(temp < 0)
        {
            temp = temp * (-1);
            negative = true;
        }

        //array [0] is 1st most sig bit, array [1] is 2nd most sig bit
        //c = 0 on 1st iteration, then c = 1 on 2nd iteration

        u0_dbg_printf("temp %% 10 = %i\n", (temp%10));

        digit = (temp % 10);
        //Convert Integer to Char:
        //Reference: https://stackoverflow.com/questions/2279379/how-to-convert-integer-to-char-in-c
        int_to_char = digit + '0';
        *(c_result + c) = int_to_char; //c_result[0] = digit[0] = 12 % 10 = 2

        u0_dbg_printf("c_result[%i] = %c\n", c, *(c_result + c));

        temp = temp / 10; //12/10 = 1.2 = 1: remove rightmost least sig digit

        u0_dbg_printf("reduce temp = %i\n", temp);

        c = c + 1;
    }

    //make sure to put negative sign in the last index after last number was stored
    //since I am going to print each number in char array in reverse order of char array result
    //Ex 5-7 = -2, so if I store '-' in a[0], then '2' in a[1] and print in reverse order,
    //I will print a[1], then a[0] resulting in "2-" instead of "-2"
    if(negative == true)
    {
        *(c_result + c) = '-';
        negative = false; //set flag back to false, so negative sign added to just the beginnging
    }

    c = 0;
}

void vUser(void *params)
{
    /*Constants*/
    const char OPERATOR = '*';
    const char OPERAND1 = '3', OPERAND2 = '7';

    /* Local Variables and Objects*/
    UARTDriver *uart_user = &uart;
    char result[MAX_ELEMENTS] = {};

    /* Init */

    while(1)
    {
        u0_dbg_printf("vUser UART-%i Tx Data:\n", (uart_user->m_uart_pc + 2));
        /* once the transfer executes and data stored into thread holding reg,
         * hardware tx_fifo & tx_shift_reg automate the data being pushed onto tx line*/
        uart_user->transfer(OPERAND1); //if I use number > ones place, then tens place and so on will be cut off
        uart_user->transfer(OPERAND2);
        uart_user->transfer(OPERATOR);
        vTaskDelay(200);

        u0_dbg_printf("vUser UART-%i Rx Result from vALU:\n", (uart_user->m_uart_pc + 2));

        // User will receive the result from the ALU task
        result[0] = uart_user->receive(); //c_result[0]
        result[1] = uart_user->receive(); //c_result[1]
        vTaskDelay(200);

        //Prints char array integer result without leading zeros to terminal
        printCharArrResult(result, OPERAND1, OPERAND2, OPERATOR, MAX_ELEMENTS);

        vTaskDelay(600);
    }

}



//Computes math operations in ASCII
void vALU(void *params)
{
    /*Pass in params*/
    //type cast params to pointer to point to objects of type UARTDriver,
    //then dereference it
//    UARTDriver uart_rx = *((UARTDriver*)(params));

    UARTDriver *uart_alu = &uart;

    /* Local Variables and Objects*/
    char data_rx[MAX_ELEMENTS] = {};
    char c_result[MAX_ELEMENTS] = {};

    //declare: asiiToInt variables
    //reference: https://www.programmingsimplified.com/c/source-code/c-program-convert-string-to-integer-without-using-atoi-function
    int32_t n[MAX_ELEMENTS] = {0};
    int32_t i_result = 0;

    enum operation
    {
        ADDITION = '+', //Decimal value 43 for char '+' symbol
        SUBTRACTION = '-', //Decimal value 45 for char '-' symbol
        MULTIPLICATION = '*', //Decimal value 42 for char '*' symbol
    };

    /* Init */
    while(1)
    {
        u0_dbg_printf("vALU UART-%i:\n", (uart_alu->m_uart_pc + 2));
        /* once the receive executes, hardware rx_shift_reg auto shifts data into
         * the rx_fifo, which is stored into Receive Buffer Reg*/
        data_rx[0] = uart_alu->receive();
        data_rx[1] = uart_alu->receive();
        data_rx[2] = uart_alu->receive();

        i_result = 0;
        //compute size of array, so I can reinitialize it
        //reference: https://stackoverflow.com/questions/37538/how-do-i-determine-the-size-of-my-array-in-c

        u0_dbg_printf("initial c_result: ");

        for(uint32_t i = 0; i < MAX_ELEMENTS; ++i)
        {
            n[i] = 0;
            c_result[i] = '0';
        }

        //passes in the address of the array
        printCharArr(c_result, MAX_ELEMENTS);

        //compute: ASCII To Integer
        //reference: https://www.programmingsimplified.com/c/source-code/c-program-convert-string-to-integer-without-using-atoi-function
        //convert ascii data_rx[0] to integer, then store to n[0]
        n[0] = n[0] * 10 + data_rx[0] - '0';

        //convert ascii data_rx[1] to integer, then store to n[1]
        n[1] = n[1] * 10 + data_rx[1] - '0';

        switch(data_rx[2])
        {
            case ADDITION:
                i_result = n[0] + n[1];
                break;
            case SUBTRACTION:
                i_result = n[0] - n[1];
                break;
            case MULTIPLICATION:
                i_result = n[0] * n[1];
                break;
            default:

                u0_dbg_printf("Math Operation Not Supported.\n");

                break;
        }
        //Store integer digits of i_result into char array c_result
        wIntToCharArr(i_result, c_result, MAX_ELEMENTS);

        //Send the result back to the user's 7 seg display
        uart_alu->transfer(c_result[0]);
        uart_alu->transfer(c_result[1]);

        vTaskDelay(1000);
    }
}

void vTestIntRx(void *params)
{
    UARTDriver *uart_test_rx = &uart;
    char byte = 'x';
    while(1)
    {
        if(xQueueReceive(uart_test_rx->rx_fifo, &byte, 500) != pdTRUE)
        {
            u0_dbg_printf("Item wasn't received from queue after 500ms\n");
        }
        else
        {
            u0_dbg_printf("Got %c from my UART-%i. Interrupt driven UART Rx done.\n", byte, (uart_test_rx->m_uart_pc + 2));
        }
    }
}

//Successfully performs math operations on two numbers using UART-2 or UART-3
//vUser
void runUARTDriverTest(void)
{

    uint32_t baud_rate = 9600; //UART bits per second

    uart.init(baud_rate, uart.CHAR_LENGTH_8BIT);

    //Set STACK_SIZE
    const uint32_t STACK_SIZE = 2048;
    const uint32_t Priority_Low = 1;

    xTaskCreate(vUser, "user", STACK_SIZE, NULL, Priority_Low, NULL);
    xTaskCreate(vALU, "alu", STACK_SIZE, NULL, Priority_Low, NULL);
}

//UART-3
//Only one task will run on one board: runUser or runALU
void runUser(void)
{

    uint32_t baud_rate = 9600; //UART bits per second

    uart.init(baud_rate, uart.CHAR_LENGTH_8BIT);

    //Set STACK_SIZE
    const uint32_t STACK_SIZE = 2048;
    const uint32_t Priority_Low = 1;

    xTaskCreate(vUser, "user", STACK_SIZE, NULL, Priority_Low, NULL);
    vTaskStartScheduler();
}

//UART-2
void runALU(void)
{

    uint32_t baud_rate = 9600; //UART bits per second

    uart.init(baud_rate, uart.CHAR_LENGTH_8BIT);

    //Set STACK_SIZE
    const uint32_t STACK_SIZE = 2048;
    const uint32_t Priority_Low = 1;

    xTaskCreate(vALU, "alu", STACK_SIZE, NULL, Priority_Low, NULL);
    vTaskStartScheduler();
}
