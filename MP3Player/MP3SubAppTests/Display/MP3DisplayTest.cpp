/*
 * MP3Display.cpp
 *
 *  Created on: Apr 22, 2018
 *      Author: james
 */

#include <MP3Player/MP3SubAppTests/Display/MP3DisplayTest.h>

/**
 * Graphic LCD Application:
 * - 84 x 48 pixel black and white LCD
 * The Pinout:
 * 1. VCC - Positive Power Supply (In) = supply range is btwn 2.7V - 3.3V
 * 2. GND - Ground (In)
 * 3. SCE - Chip Select (In) = Active Low
 * 4. RST - Reset (In) = Active Low
 * 5. D/C - Mode Select (In) = Select btwn command mode(low) and data mode(high)
 * 6. DN(MOSI) - Serial Data In (In)
 * 7. SCLK - Serial Clock (In)
 * 8. LED - LED backlight supply (In) = Max voltage supply is 3.3V, can be any PWM pin
 */

/**
 * Send command byte to LCD Display
 * Reference: Leveraged PCD8544 Datasheet Figure 10, Pg 12
 */
void sendCommand(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, uint8_t *cmd)
{
    //u0_dbg_printf("sending command byte\n");
    //D/C set LOW to specify Command byte will be sent on MOSI line
    dc->setLow();

    //u0_dbg_printf("is DC pin set LOW? %i\n", dc->getLevel());

    //Set SCE_ pin LOW to select LCD Display
    sce->setLow();
    //u0_dbg_printf("is SCE_ pin Asserted (LOW)? %i\n", sce->getLevel());
    ssp->transfer(*cmd);//send command byte over DIN pin
    sce->setHigh();
    //u0_dbg_printf("is SCE_ pin DeAsserted (HIGH)? %i\n", sce->getLevel());
}

/**
 * Send command bytes to LCD Display
 * Reference: Leveraged PCD8544 Datasheet Figure 11
 */
void sendCommandBytes(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, uint8_t *cmd, uint16_t size)
{
    for(uint8_t i = 0; i < size; i++)
    {
        sendCommand(ssp, sce, dc, cmd);
    }
}


/**
 * Send data byte to LCD Display RAM
 * Reference: Leveraged PCD8544 Datasheet Figure 10
 */
void sendData(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, uint8_t *data)
{
    //u0_dbg_printf("sending data byte\n");
    //D/C set HIGH to specify data byte will be sent on MOSI line
    dc->setHigh();

    //u0_dbg_printf("is DC pin set HIGH? %i\n", dc->getLevel());


    //Set SCE_ pin LOW to select LCD Display
    sce->setLow();
    //u0_dbg_printf("is SCE_ pin Asserted (LOW)? %i\n", sce->getLevel());
    ssp->transfer(*data); //send data byte MSB(DB7) to LSB(DB0)
    sce->setHigh();
    //u0_dbg_printf("is SCE_ pin DeAsserted (HIGH)? %i\n", sce->getLevel());
}

/**
 * Send data bytes to LCD Display
 * Reference: Leveraged PCD8544 Datasheet Figure 11
 */
void sendDataBytes(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, uint8_t *data, uint16_t size)
{
    //u0_dbg_printf("size = %i\n", size);
    for(uint8_t i = 0; i < size; i++)
    {
        sendData(ssp, sce, dc, data);
    }
}

/**
 * Performs hardware RESET on LCD Display
 * Reference: Leveraged PCD8544 Datasheet Figure 13
 */
void hardReset(LabGPIO *sce, LabGPIO *rst)
{
    //u0_dbg_printf("Hard Resetting LCD Display\n");
    //Set SCE_ pin LOW to select LCD Display
    sce->setLow();

    //u0_dbg_printf("is SCE_ pin Asserted (LOW)? %i\n", sce->getLevel());

    //Set RES_ pin LOW, then HIGH to reset LCD Display
    rst->setLow();
    //u0_dbg_printf("is RST_ pin Asserted (LOW)? %i\n", rst->getLevel());
    rst->setHigh();
    //u0_dbg_printf("is RST_ pin DeAsserted (HIGH)? %i\n", rst->getLevel());

    //Set SCE_ pin HIGH to select LCD Display
    sce->setHigh();
}

/**
 * Instruction Set:
 */

/**
 * nop() - performs no operation
 * DC set Low, Command Byte = 00000000
 */
void nop(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc)
{
    uint8_t command = 0x00; //DB7 to DB0 = 00000000
    sendCommand(ssp, sce, dc, &command);
}

/*
 * functionSet() - performs power down control; entry mode;
 * extended instruction set control (H)
 * @param function_specified
 * - Takes in command byte: 0 0 1 0 0 PD V H
 */
void functionSet(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, uint8_t function_specified)
{
    sendCommand(ssp, sce, dc, &function_specified);
}

/**
 * From Table 1:
 * displayControl() - sets the display configuration
 * @param display_configuration
 * - Takes in command byte: 0 0 0 0 1 D 0 E
 * To See different display Configurations, check Table 2:
 * Change D and E to configure display:
 * 00 = display blank
 * 10 = normal mode
 * 01 = all display segments on
 * 11 = inverse video mode
 */
void displayControl(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, uint8_t display_configuration)
{
    //the first three bits starting right to left
    //H = 0; means use basic instruction set
    //V = 0; means horizontal addressing
    //PD = 0; means chip is active
    //cmd = 0b00100000 = 0x20
    uint8_t cmd = 0x20;
    functionSet(ssp, sce, dc, cmd);

    sendCommand(ssp, sce, dc, &display_configuration);
}

/**
 * In Table 1:
 * setYAddressRam() - sets Y-address of RAM; 0 <= Y <= 5
 * @param set_y_address
 * - Takes in command byte: 0 1 0 0 0 Y2 Y1 Y0
 *
 * - Y2 Y1 Y0 = 2^3 = 8
 */
void setYAddressRam(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, uint8_t set_y_address)
{
    //the first three bits starting right to left
    //H = 0; means use basic instruction set
    //V = 0; means horizontal addressing
    //PD = 0; means chip is active
    //cmd = 0b00100000 = 0x20
    uint8_t cmd = 0x20;
    functionSet(ssp, sce, dc, cmd);

    sendCommand(ssp, sce, dc, &set_y_address);
}

/**
 * In Table 1:
 * setXAddressRam() - sets X-address of RAM; 0 <= X <= 83
 * @param set_x_address
 * - Takes in command byte: 1 X6 X5 X4 X3 X2 X1 X0
 *
 * - X6 X5 X4 X3 X2 X1 X0 = 2^7 = 128
 */
void setXAddressRam(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, uint8_t set_x_address)
{
    //the first three bits starting right to left
    //H = 0; means use basic instruction set
    //V = 0; means horizontal addressing
    //PD = 0; means chip is active
    //cmd = 0b00100000 = 0x20
    uint8_t cmd = 0x20;
    functionSet(ssp, sce, dc, cmd);

    sendCommand(ssp, sce, dc, &set_x_address);
}

/**
 * In Table 1:
 * controlTemperature() - set Temperature Coefficient (TCx)
 * @param set_temp_coeff
 * - Takes in command byte: 0 0 0 0 0 1 TC1 TC0
 * To See different display Configurations, check Table 2:
 * Change TC1 and TC0 to configure display:
 * 00 = V_LCD temperature coefficient 0
 * 01 = V_LCD temperature coefficient 1
 * 10 = V_LCD temperature coefficient 2
 * 11 = V_LCD temperature coefficient 3
 */
void controlTemperature(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, uint8_t set_temp_coeff)
{
    //the first three bits starting right to left
    //H = 1; means use extended instruction set
    //V = 0; means horizontal addressing
    //PD = 0; means chip is active
    //cmd = 0b00100001 = 0x21
    uint8_t cmd = 0x21;
    functionSet(ssp, sce, dc, cmd);

    sendCommand(ssp, sce, dc, &set_temp_coeff);
}

/**
 * setBiasSystem() - set Bias System (BSx)
 * @param set_bias_system
 * - Takes in command byte: 0 0 0 1 0 BS2 BS1 BS0
 */
void setBiasSystem(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, uint8_t set_bias_system)
{
    //the first three bits starting right to left
    //H = 1; means use extended instruction set
    //V = 0; means horizontal addressing
    //PD = 0; means chip is active
    //cmd = 0b00100001 = 0x21
    uint8_t cmd = 0x21;
    functionSet(ssp, sce, dc, cmd);

    sendCommand(ssp, sce, dc, &set_bias_system);
}

/**
 * setVop() - write Vop to register
 * @param write_vop_to_reg
 * - Takes in command byte: 1 Vop6 Vop5 Vop4 Vop3 Vop2 Vop1 Vop0
 */
void setVop(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, uint8_t write_vop_to_reg)
{
    //the first three bits starting right to left
    //H = 1; means use extended instruction set
    //V = 0; means horizontal addressing
    //PD = 0; means chip is active
    //cmd = 0b00100001 = 0x21
    uint8_t cmd = 0x21;
    functionSet(ssp, sce, dc, cmd);

    sendCommand(ssp, sce, dc, &write_vop_to_reg);
}

/**
 * setContrast() - set LCD Vop to a value between 0 and 127
 * 40-60 is usually a good range
 * Leverages setVop() function
 */
void setContrast(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, uint8_t contrast)
{
    setVop(ssp, sce, dc, ( 0x80 | contrast ) );
}

/**
 * jumpToXY() - directly commands LCD to jump to a specific X, Y coordinate
 */
void jumpToXY(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, uint8_t x, uint8_t y)
{
    //Column (Default is 0x80, which is 0 on X coordinate)
    setXAddressRam(ssp, sce, dc, ( 0x80 | x ) );
    //Row (Default is 0x40, which is 0 on Y coordinate)
    setYAddressRam(ssp, sce, dc, ( 0x40 | y) );
}


/**
 * clearDisplay() - clears the entire display either white (0) or black (1)
 */
void clearDisplay(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, bool black_white)
{
    for(uint16_t i = 0; i < (LCD_WIDTH * LCD_HEIGHT / 8); i++)
    {
        if(black_white)
        {
            displayMap[i] = 0xFF; //insert black pixel
        }
        else
        {
            displayMap[i] = 0x00; //insert white pixel (clear pixel pigment)
        }
    }

    // Draw on the display what was loaded into the displayMap to clear the display
    // Start at (X, Y) coordinate (0, 0)
    jumpToXY(ssp, sce, dc, 0, 0);

    for(uint16_t i = 0; i < (LCD_WIDTH * LCD_HEIGHT / 8); i++)
    {
        sendData(ssp, sce, dc, &displayMap[i]);
    }
}

/**
 * setPixel() - sets a pixel on displayMap to preferred color:
 * black = 1, white = 0
 */
void setPixel(uint8_t x, uint8_t y, bool black_white)
{
    //First, check the coordinate is in range
    // X is greater than equal to 0, less than LCD_WIDTH,
    // Y is greater than equal to 0, less than LCD_HEIGHT
    if(( x >= 0 ) && ( x < LCD_WIDTH ) && ( y >= 0 ) && ( y < LCD_HEIGHT ))
    {
        uint8_t shift = y % 8; //return remainder to get row location to set pixel

        if(black_white) //if black, set the bit
        {
            //x = column, y = row, LCD_WIDTH = total columns
            //set_pixel_in_column = [ column + (row/8)*total_columns ]
            //shift 1 down to certain row to set pixel
            displayMap[x + (y/8)*LCD_WIDTH] |= 1 << shift;
        }
        else //if white, clear the bit
        {
            //shift 0 down to certain row to clear pixel
            displayMap[x + (y/8)*LCD_WIDTH] &= ~(1 << shift);
        }
    }
}

/**
 * Draw character for 8 x 5 pixel space onto the display
 *
 *
 * Currently doesn't account for number of rows per character
 */
void writeCharacter(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, uint8_t x, uint8_t y, char character, bool black_white)
{
    uint8_t column = 0x00;
    // 5 columns (x) per character
    for(uint8_t col = 0; col < 5; col++)
    {
        //store data byte sequence to generate ascii onto display[0...4]
        //offset by 0x20, so we subtract the character index by 0x20
        column = ascii[character - 0x20][col];


        // 8 rows (y) per character
        for (uint8_t row = 0; row < 8; row++)
        {
            if(column & (1 << row))// column[0] test against row[0], if bit is set, then set pixel
            {
                //set pixel in X, Y coordinate location and draw ascii char part
                //eventually until the whole ascii char has been drawn for 8 x 5 space
                setPixel((x + col), (y + row), black_white);
            }
            else // if bit isn't set, then clear pixel
            {
                setPixel((x + col), (y + row), !black_white);
            }
        }
    }
    for(uint16_t i = 0; i < (LCD_WIDTH * LCD_HEIGHT / 8); i++)
    {
        sendData(ssp, sce, dc, &displayMap[i]);
    }
}

void writeString(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, uint8_t x, uint8_t y, const char *C_String, bool black_white)
{
    char letter = ' ';
    for(uint8_t i = 0; *(C_String + i) != '\0'; i++)
    {
        letter = *(C_String + i);

        writeCharacter(ssp, sce, dc, x, y, letter, black_white);

        //move along X-axis 5 columns to the right to create
        //a space between the prior char and the next char
        //The reason is because each char takes up 5 columns
        x = x + 5;

        //clear the row (8 spaces along Y-axis) column (X-axis) between the prior char and the next char
        for(uint8_t row = y; row < y + 8; row++)
        {
            setPixel(x, row, !black_white);
        }

        //move along X-axis 1 column to the right, so we can write the next char
        x = x + 1;

        //check if theres not enough room for 5 column char on current row
        //if true "wrap around" to next row starting at column 0
        if(x > (LCD_WIDTH - 5))
        {
            x = 0; //start at column 0
            y = y + 8; //start at next row
        }
    }
}

/**
 * Initialize LCD Display
 */
void initDisplay(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, LabGPIO *rst, PWMDriver *light)
{
    //8.1 Initialization
    //Immediately following power-on, the contents of all internal
    //registers and of the DDRAM are undefined

    // An external RES(or RST)_ pulse must be applied
    // So, all internal registers are reset at pad 31
    // But, the RAM contents are still undefined
    hardReset(sce, rst);

    light->setDutyCycle(light->PWM_PIN_2_3, 75); //activate backlight

    //u0_dbg_printf("performed hardware reset\n");

    //set Vop to +16 * b[V] (Contrast)
    //write_vop_to_reg = 0b10010000 = 0x90
    //set_lcd_vop = 0b10110000 = 0xB0
    uint8_t set_lcd_vop = 0xB0;
    setVop(ssp, sce, dc, set_lcd_vop);

    //u0_dbg_printf("set Vop to +16 * b[V]\n");

    //set temp coefficient
    //temp_coeff = 0b00000100 = 0x04;
    uint8_t set_temp_coeff = 0x04;
    controlTemperature(ssp, sce, dc, set_temp_coeff);

    //LCD bias mode 1:48
    //set_bias_system = 0b00010100 = 0x14;
    uint8_t set_bias_system = 0x14;
    setBiasSystem(ssp, sce, dc, set_bias_system);

    //set display control to normal mode (D = 1 and E = 0)
    //Takes in command byte: 0 0 0 0 1 D 0 E
    //display_configuration = 0b00001100 = 0x0C
    uint8_t display_configuration = 0x0C;
    displayControl(ssp, sce, dc, display_configuration);

    //u0_dbg_printf("configured display to be in normal mode\n");

}

/**
 * Displays some characters (P, H, etc) on the screen
 * and performs inverse video mode
 */
void play_simple_lcd_demo(SSPDriver *ssp, LabGPIO *sce, LabGPIO *dc, PWMDriver *light)
{
    //vTaskDelay(3000); //3 seconds
    light->setDutyCycle(light->PWM_PIN_2_3, 75); //activate backlight

    //data write Y and X are initialized to 0 by default, so they aren't set here
    //data_byte = 0b00011111 = 0x1F
    uint8_t data_byte = 0x1F;
    sendData(ssp, sce, dc, &data_byte); //letter "l" should be written to display

    //u0_dbg_printf("did data write for 'l' to display\n");

    //vTaskDelay(3000); //3 seconds

    //data write "F" to display
    //data_byte = 0b00000101 = 0x05
    data_byte = 0x05;
    sendData(ssp, sce, dc, &data_byte);

    //u0_dbg_printf("did data write for 'F' to display\n");

    //vTaskDelay(3000);

    //data write "P" to display
    //data_byte = 0b00000111 = 0x07
    data_byte = 0x07;
    sendData(ssp, sce, dc, &data_byte);

    //u0_dbg_printf("did data wite for 'P' to display\n");

    //vTaskDelay(3000);

    //move the cursor to the right by 2, still display "P"
    //data_byte = 0b00000000;
    data_byte = 0x00;
    sendData(ssp, sce, dc, &data_byte);

    //u0_dbg_printf("did data write for 'P' to stay displayed\n");

    //vTaskDelay(3000);

    //data write "P I" to display
    //data_byte = 0b00011111 = 0x1F
    data_byte = 0x1F;
    sendData(ssp, sce, dc, &data_byte);

    //u0_dbg_printf("did data write for 'P I' to display\n");

    //vTaskDelay(3000);

    //data write "P half H" to display
    //data_byte = 0b00000100 = 0x04
    data_byte = 0x04;
    sendData(ssp, sce, dc, &data_byte);

    //u0_dbg_printf("did data write for 'P half-H' to display\n");

    //vTaskDelay(3000);

    //data write "P H" to display
    //data_byte = 0b00011111 = 0x1F
    data_byte = 0x1F;
    sendData(ssp, sce, dc, &data_byte);

    //u0_dbg_printf("did data write for 'P H' to display\n");

    //vTaskDelay(5000);

    //Display control; set inverse video mode (D = 1 and E = 1)
    //Takes in command byte: 0 0 0 0 1 D 0 E
    //display_configuration = 0b00001101 = 0x0D
    uint8_t display_configuration = 0x0D;
    displayControl(ssp, sce, dc, display_configuration);

    //u0_dbg_printf("configured display to show 'P H' in inverse video mode\n");

    //vTaskDelay(5000);

    //Set X address of RAM to X's 0 coordinate; set address to '0000000'
    //Takes in command byte: 1 X6 X5 X4 X3 X2 X1 X0
    //set_x_address = 0b10000000 = 0x80
    uint8_t set_x_address = 0x80;
    setXAddressRam(ssp, sce, dc, set_x_address);

    //u0_dbg_printf("set X Address RAM to 0 X coordinate\n");

    //vTaskDelay(5000);

    //data write "half-P H" to display
    //data_byte = 0b00000000 = 0x00
    data_byte = 0x00;
    sendData(ssp, sce, dc, &data_byte);

    //u0_dbg_printf("did data write for 'half-P H' to display\n");
}

void testDisplay(void *params)
{
    //u0_dbg_printf("Called testDisplay task\n");
    //Setup the pins for LCD Display
    //SSP1
    SSPDriver lcd_display;
    PWMDriver light;
    //uint32_t pwm1_1_duty_cycle = 0;
    LabGPIO dc(2, 2); //P2.2 is data/command pin
    LabGPIO rst(2, 1); //P2.1 is reset pin
    LabGPIO sce(2, 0); //P2.0 is chip select (SCE_) pin

    dc.setAsOutput(); //Configure DC pin as output pin
    rst.setAsOutput(); //Configure RESET pin as output pin
    sce.setAsOutput(); //Configure SCE pin as output pin
    //u0_dbg_printf("config dc, rst, sce as Output\n");

    //Constants
    const uint8_t bits_to_transfer = lcd_display.TRANSFER_8_BITS;
    const uint8_t cpsdvsr = 8; //prescale divider
    //SCK1, DN(MOSI1) are leveraged
    lcd_display.init(lcd_display.SSP1, bits_to_transfer, lcd_display.SPI, cpsdvsr);

    // Set PWM Freq to 1KHz will cause LED to Dim
    light.pwmInitSingleEdgeMode(1000);
    light.pwmSelectPin(light.PWM_PIN_2_3);
    //u0_dbg_printf("initialized ssp1 peripheral\n");

    //Initialize the LCD Display
    initDisplay(&lcd_display, &sce, &dc, &rst, &light);
    //Good values range from 40-60
    setContrast(&lcd_display, &sce, &dc, 40);

    vTaskDelay(2000);

    //Clears the LCD Display pixels
    clearDisplay(&lcd_display, &sce, &dc, WHITE);

//    //Writes ascii character onto display in 8 x 5 pixel space at top left
//    writeCharacter(&lcd_display, &sce, &dc, 0, 0, 'C', BLACK);
//
//    //Writes ascii character onto display in 8 x 5 pixel space at bottom left
//    writeCharacter(&lcd_display, &sce, &dc, 0, 40, 'M', BLACK);
//
//    //Writes ascii character onto display in 8 x 5 pixel space at top right
//    writeCharacter(&lcd_display, &sce, &dc, 78, 0, 'P', BLACK);
//
//    //Writes ascii character onto display in 8 x 5 pixel space at bottom right
//    writeCharacter(&lcd_display, &sce, &dc, 78, 40, 'E', BLACK);

    //Writes a C-String to the LCD Display
    char track1[] = "Track001", track2[] = "Track002", arrow[] = "<--";
    writeString(&lcd_display, &sce, &dc, 0, 0, track1, BLACK);

    writeString(&lcd_display, &sce, &dc, 0, 8, track2, BLACK);

//    if(!(LPC_GPIO1->FIOPIN & (1 << 9)))
//    {
//        vTaskDelay(3000);
//        writeString(&lcd_display, &sce, &dc, ( (sizeof(track1)/sizeof(track1[0])) * 5) + 2, 0, arrow, WHITE);
//    }
//    else if(!(LPC_GPIO1->FIOPIN & (1 << 9)))
//    {
//        vTaskDelay(3000);
//        writeString(&lcd_display, &sce, &dc, ( (sizeof(track1)/sizeof(track1[0])) * 5) + 2, 8, arrow, WHITE);
//    }



    while(1)
    {
        if(LPC_GPIO1->FIOPIN & (1 << 9))
        {
            writeString(&lcd_display, &sce, &dc, 0, 0, track1, WHITE);
            writeString(&lcd_display, &sce, &dc, 0, 8, track2, BLACK);
        }
        else if(LPC_GPIO1->FIOPIN & (1 << 10))
        {
            writeString(&lcd_display, &sce, &dc, 0, 8, track2, WHITE);
            writeString(&lcd_display, &sce, &dc, 0, 0, track1, BLACK);
        }
//        printCharacters(&light, &lcd_display, &sce, &dc, 'A');
       // play_simple_lcd_demo(&lcd_display, &sce, &dc, &light);
        vTaskDelay(3000);
    }
}

void runDisplay(void)
{
    //u0_dbg_printf("Called runDisplay() \n");
    xTaskCreate(testDisplay, "testDisplay", 2048, NULL, 1, NULL);

    //we utilize terminal task to execute our suspend and resume producer(getLightData) and consumer(pullLightData) tasks
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    scheduler_start(); ///< This shouldn't return
    //vTaskStartScheduler();
}
