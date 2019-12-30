/*
 * LCDDisplayNokia5110.h
 *
 *  Created on: Apr 19, 2018
 *      Author: james
 */

#ifndef LCDDISPLAYNOKIA5110_H_
#define LCDDISPLAYNOKIA5110_H_

#include "printf_lib.h"
//Communicate with LCD Display over SPI0
#include "L4_IO/gpio.hpp"
#include "L2_Drivers/ssp1.h"
#include "L2_Drivers/lpc_pwm.hpp"


// Referenced "PCD8544" datasheet
// Link: https://www.sparkfun.com/datasheets/LCD/Monochrome/Nokia5110.pdf
// Now leveraging professor's driver APIs for GPIO, SPI1, PWM, etc
class LCDDisplayNokia5110 {
private:
    //SSPDriver *ssp;
    const char *m_displayName = "Nokia5110";

    GPIO *m_sce, *m_dc, *m_reset;

    PWM *m_backlight;

    enum CMD_DATA{
        LCD_COMMAND = 0,
        LCD_DATA = 1
    };

    uint8_t m_no_op_cmd = 0x00; // command to inform display to do no operation
    uint8_t m_basic_instr_cmd = 0x20; //set function to use basic instruction set
    uint8_t m_extended_instr_cmd = 0x21; //set function to use extended instruction set

    //Data Member (LCD_WIDTH/LCD_HEIGHT) of the class shared by all objects
    static constexpr uint8_t LCD_WIDTH = 84; // X-coordinates go wide (column)
    static constexpr uint8_t LCD_HEIGHT = 48; //Y-coordinates go high (row)

    /**
     * ASCII Table:
     * - Each row by 5 columns contain hex values that represent pixels
     * for a ASCII Character that is 8 pixels HIGH and 5 pixels WIDE
     *
     * - Each byte in a row represents one vertical column of a character,
     * 8 pixels HIGH
     *
     * - 5 bytes per character
     *
     * (C++ Primer 5th Edition, Lippman: 7.6 Static Class Members)
     * ascii table is associated with the LCDDisplayNokia5110 class
     * ascii table is shared by all LCDDisplayNokia5110 objects
     * can initialize static member in-class cause it has constexpr
     */
    static constexpr uint8_t ascii[][5] = {
            // The first 32 characters are ignored [0x00 - 0x19]
            // since they're non-displayable control characters
            {0x00, 0x00, 0x00, 0x00, 0x00}, // 0x20
            {0x00, 0x00, 0x5f, 0x00, 0x00}, // 0x21 !
            {0x00, 0x07, 0x00, 0x07, 0x00}, // 0x22 "
            {0x14, 0x7f, 0x14, 0x7f, 0x14}, // 0x23 #
            {0x24, 0x2a, 0x7f, 0x2a, 0x12}, // 0x24 $
            {0x23, 0x13, 0x08, 0x64, 0x62}, // 0x25 %
            {0x36, 0x49, 0x55, 0x22, 0x50}, // 0x26 &
            {0x00, 0x05, 0x03, 0x00, 0x00}, // 0x27 '
            {0x00, 0x1c, 0x22, 0x41, 0x00}, // 0x28 (
            {0x00, 0x41, 0x22, 0x1c, 0x00}, // 0x29 )
            {0x14, 0x08, 0x3e, 0x08, 0x14}, // 0x2a *
            {0x08, 0x08, 0x3e, 0x08, 0x08}, // 0x2b +
            {0x00, 0x50, 0x30, 0x00, 0x00}, // 0x2c ,
            {0x08, 0x08, 0x08, 0x08, 0x08}, // 0x2d -
            {0x00, 0x60, 0x60, 0x00, 0x00}, // 0x2e .
            {0x20, 0x10, 0x08, 0x04, 0x02}, // 0x2f /
            {0x3e, 0x51, 0x49, 0x45, 0x3e}, // 0x30 0
            {0x00, 0x42, 0x7f, 0x40, 0x00}, // 0x31 1
            {0x42, 0x61, 0x51, 0x49, 0x46}, // 0x32 2
            {0x21, 0x41, 0x45, 0x4b, 0x31}, // 0x33 3
            {0x18, 0x14, 0x12, 0x7f, 0x10}, // 0x34 4
            {0x27, 0x45, 0x45, 0x45, 0x39}, // 0x35 5
            {0x3c, 0x4a, 0x49, 0x49, 0x30}, // 0x36 6
            {0x01, 0x71, 0x09, 0x05, 0x03}, // 0x37 7
            {0x36, 0x49, 0x49, 0x49, 0x36}, // 0x38 8
            {0x06, 0x49, 0x49, 0x29, 0x1e}, // 0x39 9
            {0x00, 0x36, 0x36, 0x00, 0x00}, // 0x3a :
            {0x00, 0x56, 0x36, 0x00, 0x00}, // 0x3b ;
            {0x08, 0x14, 0x22, 0x41, 0x00}, // 0x3c <
            {0x14, 0x14, 0x14, 0x14, 0x14}, // 0x3d =
            {0x00, 0x41, 0x22, 0x14, 0x08}, // 0x3e >
            {0x02, 0x01, 0x51, 0x09, 0x06}, // 0x3f ?
            {0x32, 0x49, 0x79, 0x41, 0x3e}, // 0x40 @
            {0x7e, 0x11, 0x11, 0x11, 0x7e}, // 0x41 A
            {0x7f, 0x49, 0x49, 0x49, 0x36}, // 0x42 B
            {0x3e, 0x41, 0x41, 0x41, 0x22}, // 0x43 C
            {0x7f, 0x41, 0x41, 0x22, 0x1c}, // 0x44 D
            {0x7f, 0x49, 0x49, 0x49, 0x41}, // 0x45 E
            {0x7f, 0x09, 0x09, 0x09, 0x01}, // 0x46 F
            {0x3e, 0x41, 0x49, 0x49, 0x7a}, // 0x47 G
            {0x7f, 0x08, 0x08, 0x08, 0x7f}, // 0x48 H
            {0x00, 0x41, 0x7f, 0x41, 0x00}, // 0x49 I
            {0x20, 0x40, 0x41, 0x3f, 0x01}, // 0x4a J
            {0x7f, 0x08, 0x14, 0x22, 0x41}, // 0x4b K
            {0x7f, 0x40, 0x40, 0x40, 0x40}, // 0x4c L
            {0x7f, 0x02, 0x0c, 0x02, 0x7f}, // 0x4d M
            {0x7f, 0x04, 0x08, 0x10, 0x7f}, // 0x4e N
            {0x3e, 0x41, 0x41, 0x41, 0x3e}, // 0x4f O
            {0x7f, 0x09, 0x09, 0x09, 0x06}, // 0x50 P
            {0x3e, 0x41, 0x51, 0x21, 0x5e}, // 0x51 Q
            {0x7f, 0x09, 0x19, 0x29, 0x46}, // 0x52 R
            {0x46, 0x49, 0x49, 0x49, 0x31}, // 0x53 S
            {0x01, 0x01, 0x7f, 0x01, 0x01}, // 0x54 T
            {0x3f, 0x40, 0x40, 0x40, 0x3f}, // 0x55 U
            {0x1f, 0x20, 0x40, 0x20, 0x1f}, // 0x56 V
            {0x3f, 0x40, 0x38, 0x40, 0x3f}, // 0x57 W
            {0x63, 0x14, 0x08, 0x14, 0x63}, // 0x58 X
            {0x07, 0x08, 0x70, 0x08, 0x07}, // 0x59 Y
            {0x61, 0x51, 0x49, 0x45, 0x43}, // 0x5a Z
            {0x00, 0x7f, 0x41, 0x41, 0x00}, // 0x5b [
            {0x02, 0x04, 0x08, 0x10, 0x20}, // 0x5c \ (keep this to escape the backslash)
            {0x00, 0x41, 0x41, 0x7f, 0x00}, // 0x5d ]
            {0x04, 0x02, 0x01, 0x02, 0x04}, // 0x5e ^
            {0x40, 0x40, 0x40, 0x40, 0x40}, // 0x5f _
            {0x00, 0x01, 0x02, 0x04, 0x00}, // 0x60 `
            {0x20, 0x54, 0x54, 0x54, 0x78}, // 0x61 a
            {0x7f, 0x48, 0x44, 0x44, 0x38}, // 0x62 b
            {0x38, 0x44, 0x44, 0x44, 0x20}, // 0x63 c
            {0x38, 0x44, 0x44, 0x48, 0x7f}, // 0x64 d
            {0x38, 0x54, 0x54, 0x54, 0x18}, // 0x65 e
            {0x08, 0x7e, 0x09, 0x01, 0x02}, // 0x66 f
            {0x0c, 0x52, 0x52, 0x52, 0x3e}, // 0x67 g
            {0x7f, 0x08, 0x04, 0x04, 0x78}, // 0x68 h
            {0x00, 0x44, 0x7d, 0x40, 0x00}, // 0x69 i
            {0x20, 0x40, 0x44, 0x3d, 0x00}, // 0x6a j
            {0x7f, 0x10, 0x28, 0x44, 0x00}, // 0x6b k
            {0x00, 0x41, 0x7f, 0x40, 0x00}, // 0x6c l
            {0x7c, 0x04, 0x18, 0x04, 0x78}, // 0x6d m
            {0x7c, 0x08, 0x04, 0x04, 0x78}, // 0x6e n
            {0x38, 0x44, 0x44, 0x44, 0x38}, // 0x6f o
            {0x7c, 0x14, 0x14, 0x14, 0x08}, // 0x70 p
            {0x08, 0x14, 0x14, 0x18, 0x7c}, // 0x71 q
            {0x7c, 0x08, 0x04, 0x04, 0x08}, // 0x72 r
            {0x48, 0x54, 0x54, 0x54, 0x20}, // 0x73 s
            {0x04, 0x3f, 0x44, 0x40, 0x20}, // 0x74 t
            {0x3c, 0x40, 0x40, 0x20, 0x7c}, // 0x75 u
            {0x1c, 0x20, 0x40, 0x20, 0x1c}, // 0x76 v
            {0x3c, 0x40, 0x30, 0x40, 0x3c}, // 0x77 w
            {0x44, 0x28, 0x10, 0x28, 0x44}, // 0x78 x
            {0x0c, 0x50, 0x50, 0x50, 0x3c}, // 0x79 y
            {0x44, 0x64, 0x54, 0x4c, 0x44}, // 0x7a z
            {0x00, 0x08, 0x36, 0x41, 0x00}, // 0x7b {
            {0x00, 0x00, 0x7f, 0x00, 0x00}, // 0x7c |
            {0x00, 0x41, 0x36, 0x08, 0x00}, // 0x7d }
            {0x10, 0x08, 0x08, 0x10, 0x08}, // 0x7e ~
            {0x78, 0x46, 0x41, 0x46, 0x78} // 0x7f DEL
    };

    /**
     * displayMap stores buffer of pixels on the display
     *
     * - 504 total bytes in this array equivalent to the number of pixels
     * that are on the 84 x 48 display
     *
     * - each byte covers an 8 pixel vertical block on the display
     *
     * - every following byte covers the next 8 pixel column until you
     * reach the right-edge of the display, then the following byte steps
     * down 8 pixels to the next row
     *
     * - everytime we want to update the display, we update this array,
     * then send it as data bytes to update display data RAM (DDRAM)
     *
     * - since the PCD8544 doesn't allow us to write individual pixels
     * at a time, we use this array to make the necessary changes
     *
     * Each object instantiated from this class will share a copy
     * of this ascii table
     */
    uint8_t displayMap[LCD_WIDTH * LCD_HEIGHT / 8] = {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (0,0)->(11,7) ~ These 12 bytes cover an 8x12 block in the left corner of the display
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (12,0)->(23,7)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, // (24,0)->(35,7)
            0xF0, 0xF8, 0xFC, 0xFC, 0xFE, 0xFE, 0xFE, 0xFE, 0x1E, 0x0E, 0x02, 0x00, // (36,0)->(47,7)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (48,0)->(59,7)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (60,0)->(71,7)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (72,0)->(83,7)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (0,8)->(11,15)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (12,8)->(23,15)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, // (24,8)->(35,15)
            0x0F, 0x1F, 0x3F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFC, 0xF8, // (36,8)->(47,15)
            0xF8, 0xF0, 0xF8, 0xFE, 0xFE, 0xFC, 0xF8, 0xE0, 0x00, 0x00, 0x00, 0x00, // (48,8)->(59,15)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (60,8)->(71,15)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (72,8)->(83,15)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (0,16)->(11,23)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (12,16)->(23,23)
            0x00, 0x00, 0xF8, 0xFC, 0xFE, 0xFE, 0xFF, 0xFF, 0xF3, 0xE0, 0xE0, 0xC0, // (24,16)->(35,23)
            0xC0, 0xC0, 0xE0, 0xE0, 0xF1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // (36,16)->(47,23)
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3E, 0x00, 0x00, 0x00, // (48,16)->(59,23)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (60,16)->(71,23)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (72,16)->(83,23)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (0,24)->(11,31)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (12,24)->(23,31)
            0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // (24,24)->(35,31)
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // (36,24)->(47,31)
            0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x1F, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, // (48,24)->(59,31)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (60,24)->(71,31)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (72,24)->(83,31)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (0,32)->(11,39)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (12,32)->(23,39)
            0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x1F, // (24,32)->(35,39)
            0x0F, 0x0F, 0x0F, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x03, 0x03, // (36,32)->(47,39)
            0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (48,32)->(59,39)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (60,32)->(71,39)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (72,32)->(83,39)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (0,40)->(11,47)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (12,40)->(23,47)
            0x00, 0x00, 0x3F, 0x1F, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, // (24,40)->(35,47)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (36,40)->(47,47)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (48,40)->(59,47)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (60,40)->(71,47)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (72,40)->(83,47) !!! The bottom right pixel!
    };

public:

    //Constants that specifies how the display will be cleared
    enum CLEAR_DISPLAY{
        WHITE = 0, //draw pixels white
        BLACK = 1 //or invert and draw pixels black
    };

    /**
     * Default Constructor
     */
    LCDDisplayNokia5110();
    /**
     * Parameter Constructor for setting up the users pins that they want
     * to interface with the LCD display
     */
    LCDDisplayNokia5110(GPIO *sce_pin, GPIO *dc_pin, GPIO *res_pin, PWM *backlight_pin);
    virtual ~LCDDisplayNokia5110();

    /**
     * Initialize pin peripherals for LCD Display
     */
    void initPinPeripherals();

    /**
     * Interface that turns the LED backlight dimming HIGH. This is done
     * by setting PWM Duty Cycle, which is the percentage of time
     * that the PWM signal is HIGH (100%) to its full period.
     *
     * The Duty Cycle is dependent on the PWM Frequency, so if the frequency
     * at initialization of PWM is set to 1Hz, the signal repeats more slowly
     * and would cause an LED to blink. If set to 1KHz, the signal repeats
     * more rapidly and will cause the LED to dim.
     *
     * Ex: A Duty Cycle = 75% means it is HIGH for 75% of its period(time)
     * and LOW for 25%
     */
    void activateLEDBackLight();

    /**
     * Turn off LED backlight dimming by setting it LOW (0%)
     */
    void deactivateLEDBackLight();

    /**
     * Interface that performs hardware RESET (active LOW) on LCD Display
     * Reference: Leveraged PCD8544 Datasheet Figure 13
     */
    void hardReset();

    /**
     * Interface that adjusts the 'sharpness' of the liquid crystal display to make
     * it clear for the user to read the text.
     *
     * @param contrast The contrast level that will be set for the display
     *
     * @note contrast levels between 40 - 60 is a good range
     */
    void setContrast(uint8_t contrast);

    /**
     * Interface that updates the entire display by transferring the changes
     * made to the 504 byte displayMap array to the display's DDRAM
     *
     */
    void updateDisplay();

    /**
     * Interface that clears the entire display either white (0) or black (1)
     * by transferring the changes made to the 504 byte displayMap array to
     * the display's DDRAM
     *
     * @param black_white The enum constants BLACK or WHITE that specify
     * how the display will be cleared
     */
    void clearDisplay(bool black_white);

    /**
     * Interface that initializes LCD Display:
     * - set up pins connected to the display
     * - set up serial peripheral interface (SPI) protocol as SPI1
     * - send commands to hardware reset display's internal register
     * - since DDRAM contents are undefined after hardware reset,
     * clear pixels on display using displayMap
     */
    bool initDisplay();

    /**
     * Interface that prints ascii character (defined in multi-dimensional
     * array ASCII table) at a specific X and Y coordinate onto the display
     *
     * @param x The X coordinate for the desired character to be printed
     * @param y The Y coordinate for the desired character to be printed
     * @param character The ascii character that will be sent to the display
     * @param black_white The color that the ascii character will appear as
     * black = 1 or white = 0 on the display
     */
    void printCharacter(uint8_t x, uint8_t y, char character, bool black_white);

    /**
     * Interface that prints a string onto the display at a specific X and Y
     * coordinate
     *
     * @param x The X coordinate for the desired c_string to be printed
     * @param y The Y coordinate for the desired c_string to be printed
     * @param *c_string The pointer to const char data types that points to
     * usually an array of characters that make up a string
     * @param black_white The color that the c_string will appear as
     * black = 1 or white = 0 on the display
     */
    void printString(uint8_t x, uint8_t y, const char *c_string, bool black_white);



private:

    /**
     * Select or de-select display using SCE chip select pin
     */

    /**
     * The SCE_ (active LOW) Chip Enable pin allows data to be clocked in
     * This input pin on LCD Display is sent an active LOW signal
     */
    void selectDisplay();
    void deselectDisplay();

    /**
     * Sets the D/C_ pin HIGH to select data transmission mode
     */
    void selectDataMode();

    /**
     * Sets the D/C_ pin LOW to select command transmission mode
     */
    void selectCommandMode();

    /**
     * There are two memory banks in the LCD:
     * - Commands
     * - Display Data RAM (DDRAM)
     */

    /**
     * Send command byte to LCD Display
     * Reference: PCD8544 Datasheet Figure 10, pg 12
     *
     * @param *cmd The command pointer holds a copy of the address of the command
     */
    void sendCommand(uint8_t *cmd);

    /**
     * Send command bytes to LCD Display
     * Reference: PCD8544 Datasheet Figure 11, pg 12
     *
     * @param *cmd The command pointer holds a copy of the address of the command array
     * @param size The number of elements of the array
     */
    void sendCommands(uint8_t *cmd, uint8_t size);

    /**
     * Send data byte MSB(DB7) to LSB(DB0) to LCD Display Data RAM (DDRAM)
     * Reference: PCD8544 Datasheet Figure 10, pg 12
     *
     * @param *data The data pointer holds a copy of the address of the data
     */
    void sendData(uint8_t *data);

    /**
     * Send data bytes to LCD Display Data RAM (DDRAM)
     * Reference: PCD8544 Datasheet "Figure 11", pg 12
     *
     * @param *data The data pointer holds a copy of the address of the data array
     * @param size The number of elements of the array
     */
    void sendDataBytes(uint8_t *data, uint8_t size);

    /**
     * Leverage sendCommand() method to build the "Instruction Set" methods:
     * Reference: PCD8544 Datasheet Table 1 and Table 2, Pg 14
     * When H = 0 or H = 1:
     * - nop()
     * - setFunction()
     *
     * When H = 0:
     * - reserved, so ignore it
     * - controlDisplay()
     * - reserved, so ignore it
     * - setYAddressDDRAM()
     * - setXAddressDDRAM()
     *
     * When H = 1:
     * - reserved, so ignore it
     * - controlTemperature()
     * - reserved, so ignore it
     * - setBiasSystem()
     * - reserved, so ignore it
     * - setVop()
     */

    /**
     * Performs no operation
     *
     * DC pin set LOW, Command Byte = 0b00000000 = 0x00
     */
    void nop();

    /**
     * Performs power down control, entry mode and can extend instruction set (H)
     *
     * @param specify_function The specific functions or instructions you want
     * to access, either basic instructions when H bit = 0
     * or extended instructions when H bit = 1
     *
     * @note: command byte sequence, 0 0 1 0 0 PD V H, is sent to the display and
     * we are just concerned with setting or clearing the H bit
     */
    void setFunction(uint8_t specify_function);

    /**
     * Sets the display configuration to be display blank, normal mode,
     * all display segments on or inverse video mode
     *
     * @param configure_display The display configuration controls the display mode
     *
     * @note: In the command byte sequence, 0 0 0 0 1 D 0 E, by setting and/or
     * clearing D and E bits, we can control the display modes:
     *
     * - 00 = display blank
     * - 10 = normal mode
     * - 01 = all display segments on
     * - 11 = inverse video mode
     */
    void controlDisplay(uint8_t configure_display);

    /**
     * Sets Y vector address of DDRAM, must fall between banks 0 <= Y <= 5
     * Reference: PCD8544 Datasheet "8.5 Set Y address of RAM", pg 15
     *
     * @param y_address The Y vector address used to select a bank,
     * which is a row 8-pixels HIGH
     *
     * @note: In the command byte sequence, 0 1 0 0 0 Y2 Y1 Y0, by setting and/or
     * clearing Y2 Y1 Y0 bits, we can select a particular bank:
     *
     * - 000 = bank 0
     * - 001 = bank 1
     * - 010 = bank 2
     * - 011 = bank 3
     * - 100 = bank 4
     * - 101 = bank 5
     */
    void setYAddressDDRAM(uint8_t y_address);

    /**
     * Sets X vector address of DDRAM, must fall between 0 <= X <= 83
     * Reference: PCD8544 Datasheet "7.7 Addressing", pg 9
     *
     * @param x_address The X address points to columns
     *
     * @note: In the command byte sequence, 1 X6 X5 X4 X3 X2 X1 X0, by setting and/or
     * clearing X6 X5 X4 X3 X2 X1 X0 bits, we can access certain columns
     *
     * - 10000000 = column 0
     * - 10000001 = column 1
     * - 10000010 = column 2
     * ...
     * ...
     * - 11010011 = column 83
     */
    void setXAddressDDRAM(uint8_t x_address);

    /**
     * Sets temperature coefficient (TCx) because temperature is dependent on
     * liquid crystals' viscosity
     *
     * Reference: PCD8544 Datasheet, pg 14
     *
     * @param set_temp_coeff The temperature coefficient of V_LCD can be selected
     * from four values by setting TC1 and TC0 bits
     *
     * @note: In the command byte sequence, 0 0 0 0 0 1 TC1 TC0, by setting/clearing
     * TC1 TC0 bits, we can select the appropriate temperature coefficient:
     *
     * - 00 = V_LCD temperature coefficient 0
     * - 01 = V_LCD temperature coefficient 1
     * - 10 = V_LCD temperature coefficient 2
     * - 11 = V_LCD temperature coefficient 3
     */
    void controlTemperature(uint8_t set_temp_coeff);

    /**
     * Sets Bias System (BSx) multiplex ratio rate. The rate indicates how many
     * voltage reference points are created to drive the LCD.
     *
     * For example, in the case the multiplex ratio is 1:48, resulting in 1/8 bias,
     * the optimum bias value "n" is: n = sqrt(48) - 3 = 3,928 = 4. So, LCD Bias
     * number indicates there are 4 voltage reference points that would be created.
     *
     * Reference: PCD8544 Datasheet "8.8 Bias value", pg 15, 16
     * Reference: http://www.pacificdisplay.com/lcd_multiplex_drive.htm
     *
     * @param bias_number The number of voltage reference points created to drive
     * LCD
     *
     * @note: In the command byte sequence, 0 0 0 1 0 BS2 BS1 BS0, by setting and/or
     * clearing BS2 BS1 BS0 bits,
     *
     * - 000 = n is 7 and mux rate is 1:100
     * and so on
     */
    void setBiasSystem(uint8_t bias_number);

    /**
     * Sets Vop value, which will be used in calculating the operation voltage
     * V_LCD. Equation that Vop is applied in: V_LCD = a + (Vop6 to Vop0) * b[V]
     *
     * In the PCD8544, a = 3.06, b = 0.06, which gives a program range of
     * 3.00 to 10.68 at room temperature
     *
     * Reference: PCD8544 Datasheet "8.9 Vop value", pg 16
     *
     * @param vop_value The voltage operation value that will be used to set the
     * operation voltage of the LCD display
     *
     * @note: In the command byte sequence, 1 Vop6 Vop5 Vop4 Vop3 Vop2 Vop1 Vop0,
     * by setting/clearing these Vop6 to Vop0 bits, we can alter V_LCD.
     */
    void setVop(uint8_t vop_value);

    /**
     * Leverage "Instruction Set" methods to build even simpler utility methods
     * for performing actions on the display that are more clear to the user:
     */

    /**
     * Directly commands the LCD to jump to a specific X and Y coordinate on DDRAM
     *
     * @param x The X coordinate (column pixel) that will be located on display map
     * @param y The Y coordinate (row pixel) that will be located on display map
     *
     * @note: The display is 48 rows by 84 columns, so x and y must be within:
     *
     * 0 <= x-column-pixels <= 83
     * 0 <= y-row-pixels <= 47
     */
    void jumpToXY(uint8_t x, uint8_t y);

    /**
     * Sets a pixel on the display map to be black = 1 or white = 0
     *
     * @param x The X coordinate (column pixel) on the display to set the pixel
     * @param y The Y coordinate (row pixel) on the display to set the pixel
     * @param black_white The color to set the pixel black = 1 or white = 0
     */
    void setPixel(uint8_t x, uint8_t y, bool black_white);
};

#endif /* LCDDISPLAYNOKIA5110_H_ */
