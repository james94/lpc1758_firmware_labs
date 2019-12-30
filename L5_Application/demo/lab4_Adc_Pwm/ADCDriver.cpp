/*
 * ADCDriver.cpp
 *
 *  Created on: Feb 24, 2018
 *      Author: james
 */

#include <demo/lab4_Adc_Pwm/ADCDriver.hpp>

ADCDriver::ADCDriver()
{
    // TODO Auto-generated constructor stub

}


void ADCDriver::adcInitBurstMode()
{
    // 1) Powers up ADC peripheral (UM10360 Table 46)
    LPC_SC->PCONP |= (1 << 12);

    // 2) Set peripheral clock (UM10360 Table 41)
    LPC_SC->PCLKSEL0 |= (1 << 24);

    // 3) Enable ADC, set PDN bit in AD0CR reg (UM10360 Table 532)
    LPC_ADC->ADCR |= (1 << 21); // required, so AD conversion can occur

    // 4) Disable ADGINTEN since we aren't using interrupts
    LPC_ADC->ADINTEN &= ~(1 << 8);

    // 5) Select ADC channels (Signal lvls <= 3.3v):

    LPC_ADC->ADCR |= (1 << 2); // AD0.2
    LPC_ADC->ADCR |= (1 << 3); // AD0.3
    LPC_ADC->ADCR |= (1 << 4); // AD0.4
    LPC_ADC->ADCR |= (1 << 5); // AD0.5

    // 6) Enable burst mode
    // What happens in burst mode?
    // ADC does repeated conversion up to 200KHz
    // scans selected pins set to ones in SEL field: AD0.2, AD0.3, AD0.4, AD0.5
    // one of those pins will be scanned based on which one user selects
    // first conversion after the start corrsponds to least-significant one in SEL field
    // then higher numbered 1-bits(pins) if applicable

    // How to enable burst mode? (UM10360 Table 532)
    LPC_ADC->ADCR &= ~(7 << 24); //set START bits to 000
    LPC_ADC->ADCR |= (1 << 16); //set BURST bit to 1
}

/**
* 1) Selects ADC functionality of any of the ADC pins that are ADC capable
*
* @param adc_pin_arg is the ADC_PIN enumeration of the desired pin.
*
* WARNING: For proper operation of the SJOne board, do NOT configure any pins
*           as ADC except for 0.26, 1.31, 1.30
*/
void ADCDriver::adcSelectPin(ADC_PIN adc_pin_arg)
{
    // 4) Enable ADC0 pin selected by user via PINSEL (UM10360 Ch8)
    switch(adc_pin_arg)
    {
        case ADC_PIN_0_25: // <-- Light Sensor Test
            LPC_PINCON->PINSEL1 &= ~(3 << 18);
            LPC_PINCON->PINSEL1 |= (1 << 18); // P0.25: AD0.2
            break;
        case ADC_PIN_0_26:
            LPC_PINCON->PINSEL1 &= ~(3 << 20);
            LPC_PINCON->PINSEL1 |= (1 << 20); // P0.26: AD0.3
            break;
        case ADC_PIN_1_30:
            LPC_PINCON->PINSEL1 |= (3 << 28); // P1.30: AD0.4
            break;
        case ADC_PIN_1_31:
            LPC_PINCON->PINSEL1 |= (3 << 30); // P1.31: AD0.5
            break;
        default:
            u0_dbg_printf("Invalid ADC pin\n");
            break;
    }
}

/**
* 1) Returns the voltage reading of the 12bit register of a given ADC channel
*
* @param adc_channel_arg is the number (0 through 7) of the desired ADC channel.
*/
float ADCDriver::readADCVoltageByChannel(uint8_t adc_channel_arg)
{
    float ADCVoltage = 0;
    // Check the status of AD0.2,3,4,5
    switch(adc_channel_arg)
    {
        //Checking the DONE bit in ADDR reg would be done if we were using interrupts
        case ADC_CHANNEL_2:
            //First I shift my value in reg to right by 4, so result starts at 0 to 11
            //Then I clear the 20 bits that are before the 12 result bits
            //So, the user just receives the 12 result bits from ADC Channel 2
            ADCVoltage = (LPC_ADC->ADDR2 >> 4) & 0xFFF;
            break;
        case ADC_CHANNEL_3:
            //Refer to explanation in ADC_CHANNEL_2, but think in terms of ADC_CHANNEL_3
            ADCVoltage = (LPC_ADC->ADDR3 >> 4) & 0xFFF;
            break;
        case ADC_CHANNEL_4:
            //Refer to explanation in ADC_CHANNEL_2, but think in terms of ADC_CHANNEL_4
            ADCVoltage = (LPC_ADC->ADDR4 >> 4) & 0xFFF;
            break;
        case ADC_CHANNEL_5:
            //Refer to explanation in ADC_CHANNEL_2, but think in terms of ADC_CHANNEL_3
            ADCVoltage = (LPC_ADC->ADDR5 >> 4) & 0xFFF;
            break;
        default:
            u0_dbg_printf("Invalid Channel\n");
            break;
    }

    return ADCVoltage;
}


ADCDriver::~ADCDriver()
{
    // TODO Auto-generated destructor stub
}

