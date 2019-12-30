/*
 * PWMDriver.cpp
 *
 *  Created on: Feb 24, 2018
 *      Author: james
 */

#include <demo/lab4_Adc_Pwm/PWMDriver.hpp>

PWMDriver::PWMDriver()
{
    // TODO Auto-generated constructor stub

}

/**
* 1) Select PWM functionality on all PWM-able pins.
*/
void PWMDriver::pwmSelectAllPins()
{
    /* Select All PWM Pins: 2.0-2.5 via PINSEL */
    // Reset P2.0-P2.5 pinsel4 bits to 0
    LPC_PINCON->PINSEL4 &= ~(0xFFF << 0);
    //    0101 0101 0101
    // 0x 5    5    5
    // Select PWM functionality for P2.0-P2.5 PWM1.1-PWM1.6
    LPC_PINCON->PINSEL4 |= (0x555 << 0);
}

/**
* 1) Select PWM functionality of pwm_pin_arg
*
* @param pwm_pin_arg is the PWM_PIN enumeration of the desired pin.
*/
void PWMDriver::pwmSelectPin(PWM_PIN pwm_pin_arg)
{
    u0_dbg_printf("Entered pwmSelectPin()\n");
    /* Select specific PWM Pin: 2.0-2.5 via PINSEL */
    // 0 0 0 0 0 0 0 0 0
    // 1111 1111 1111 = 0xFFF
    // Reset P2.0-P2.5 pinsel4 bits to 0

    // ERROR: Cannot clear other wire configuration
    // LPC_PINCON->PINSEL4 &= ~(0xFFF << 0);

    switch(pwm_pin_arg)
    {
        case PWM_PIN_2_0:
            LPC_PINCON->PINSEL4 &= ~(3 << 0);
            LPC_PINCON->PINSEL4 |= (1 << 0); //sel PWM1.1
            break;
        case PWM_PIN_2_1:
            LPC_PINCON->PINSEL4 &= ~(3 << 2);
            LPC_PINCON->PINSEL4 |= (1 << 2); //sel PWM1.2
            break;
        case PWM_PIN_2_2:
            LPC_PINCON->PINSEL4 &= ~(3 << 4);
            LPC_PINCON->PINSEL4 |= (1 << 4); //sel PWM1.3
            break;
        case PWM_PIN_2_3:
            LPC_PINCON->PINSEL4 &= ~(3 << 6);
            LPC_PINCON->PINSEL4 |= (1 << 6); //sel PWM1.4
            break;
        case PWM_PIN_2_4:
            LPC_PINCON->PINSEL4 &= ~(3 << 8);
            LPC_PINCON->PINSEL4 |= (1 << 8); //sel PWM1.5
            break;
        case PWM_PIN_2_5:
            LPC_PINCON->PINSEL4 &= ~(3 << 10);
            LPC_PINCON->PINSEL4 |= (1 << 10); //sel PWM1.6
            break;
    }
}

/**
*
* Initialize the PWM peripheral, its frequency, and
* set PWM channels to 0% duty cycle
*
* @param frequency_Hz is the initial frequency in Hz.
*/
void PWMDriver::pwmInitSingleEdgeMode(uint32_t frequency_Hz)
{
    u0_dbg_printf("Entered pwmInitSingleEdgeMode()\n");
    //Chapter 24: PWM
    /*1. PCONP: Power PCPWM1 Peripheral */
    LPC_SC->PCONP |= (1 << 6);

    /*2. PCLKSEL0: Set PCLK_PWM1 Peripheral Clk */
    LPC_SC->PCLKSEL0 |= (1 << 12);

    /*Table 446 Regs needed for init: TCR, MCR, PCR, LER, CTCR*/

    /*CTCR: select Timer mode*/
    LPC_PWM1->CTCR &= ~(3 << 0); //Set Timer Mode, so TC increments when PC matches Prescale Reg
    /*CTCR: Count Input Select Reset, we aren't using Counter mode*/
    LPC_PWM1->CTCR &= ~(3 << 2);

    /*PWM1MCR: Reset PWM Timer Counter if PWM Match Reg 0 matches it */
    LPC_PWM1->MCR |= (1 << 1); /*Don't care about MR[1-6] cause then TC will never reach MR0*/

    /*Three initial steps before enabling PWM Mode via TCR*/

    /*- Set PWM1 Freq, which sets (MR0), which is the PWM Rate*/
    /*- Inside setFrequency(), set PWM1 LER reg bit 0 to latch in MR0 data*/
    setFrequency(frequency_Hz);

    /*- PWM1TCR: Enable Timer and Prescale Counter for counting */
    LPC_PWM1->TCR |= (1 << 0);

    /*PWM1TCR: Enable PWM Mode*/
    LPC_PWM1->TCR |= (1 << 3);

    /*3. PWM1PCR: Set PWM 1.1 - 1.6 Single Edge Controlled mode*/
    /*via PWMSEL2-PWMSEL6 bits = 0*/
    // ~(0001 1111) = ~(0x1F)
    LPC_PWM1->PCR &= ~( (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) );

    /*4. PWM1PCR: Enable 6 PWM outputs for PWM1.1 to PWM1.6*/
    LPC_PWM1->PCR |= (1 << 9) | (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14);

    /*Set PWM Channels to 0% Duty Cycle, so all PWM channel signals will be LOW
    at start of PWM cycle, unless user changes them*/
    setDutyCycle(PWM_PIN_2_0, 0); /*PWM1.1*/
    setDutyCycle(PWM_PIN_2_1, 0); /*PWM1.2*/
    setDutyCycle(PWM_PIN_2_2, 0); /*PWM1.3*/
    setDutyCycle(PWM_PIN_2_3, 0); /*PWM1.4*/
    setDutyCycle(PWM_PIN_2_4, 0); /*PWM1.5*/
    setDutyCycle(PWM_PIN_2_5, 0); /*PWM1.6*/
}

/**
 * How to set Duty Cycle?
 * 1) Convert duty_cycle_percentage to the appropriate match register value (depends on current frequency)
 * 2) Assign the above value to the appropriate MRn register (depends on pwm_pin_arg)
 *
 * @param pwm_pin_arg is the PWM_PIN enumeration of the desired pin.
 * @param duty_cycle_percentage is the desired duty cycle percentage.
 *
 * What is Duty Cycle?
 *  - Of a certain PWM signal, it is the percentage of that signal's "on" time
 *    to the signal's full period.
 *  - Ex: If the duty cycle is 75%, it is high for 75% of its period
 *    and low for remaining 25%.
 */
void PWMDriver::setDutyCycle(PWM_PIN pwm_pin_arg, float duty_cycle_percentage)
{
    u0_dbg_printf("Entered setDutyCycle()\n");
    /*Set Duty Cycle by sequence of events:*/
    /*1. MRx: Set new Duty Cycle Percentage*/
    /*2. LER: Set bit x, so altered values will be Latched in MRx on next reset of timer*/
    switch(pwm_pin_arg)
    {
        case PWM_PIN_2_0: /*PWM1.1*/
            /*Typical sequence of events for changing timing would be:*/
            LPC_PWM1->MR1 = ( (duty_cycle_percentage/100) * LPC_PWM1->MR0 ); /*Set new Duty Cycle Percentage*/
            LPC_PWM1->LER |= (1 << 1); /*Set bit 1, so altered values will be Latched in MR1 on next reset of timer*/
            break;
        case PWM_PIN_2_1: /*PWM1.2*/
            LPC_PWM1->MR2 = ( (duty_cycle_percentage/100) * LPC_PWM1->MR0 ); /*Set new Duty Cycle Percentage*/
            LPC_PWM1->LER |= (1 << 2);
            break;
        case PWM_PIN_2_2: /*PWM1.3*/
            LPC_PWM1->MR3 = ( (duty_cycle_percentage/100) * LPC_PWM1->MR0 ); /*Set new Duty Cycle Percentage*/
            LPC_PWM1->LER |= (1 << 3);
            break;
        case PWM_PIN_2_3: /*PWM1.4*/
            LPC_PWM1->MR4 = ( (duty_cycle_percentage/100) * LPC_PWM1->MR0 ); /*Set new Duty Cycle Percentage*/
            LPC_PWM1->LER |= (1 << 4);
            break;
        case PWM_PIN_2_4: /*PWM1.5*/
            LPC_PWM1->MR5 = ( (duty_cycle_percentage/100) * LPC_PWM1->MR0 ); /*Set new Duty Cycle Percentage*/
            LPC_PWM1->LER |= (1 << 5);
            break;
        case PWM_PIN_2_5: /*PWM1.6*/
            LPC_PWM1->MR6 = ( (duty_cycle_percentage/100) * LPC_PWM1->MR0 ); /*Set new Duty Cycle Percentage*/
            LPC_PWM1->LER |= (1 << 6);
            break;
    }
    u0_dbg_printf("TC count: %i\n", LPC_PWM1->TC);
    u0_dbg_printf("MR0 max count: %i\n", LPC_PWM1->MR0);
}

/**
 * 1) Retrieve the above value from the appropriate MRn register (depends on pwm_pin_arg)
 *
 * @param pwm_ping_arg is the PWM_PIN enumeration of the desired pin.
 */
uint32_t PWMDriver::getDutyCycle(PWM_PIN pwm_pin_arg)
{
    //u0_dbg_printf("Entered getDutyCycle()\n");
    uint32_t duty_cycle = 0;
    switch(pwm_pin_arg)
    {
        case PWM_PIN_2_0: /*PWM1.1*/
            duty_cycle = LPC_PWM1->MR1; /*Get Duty from Match Reg*/
            break;
        case PWM_PIN_2_1: /*PWM1.2*/
            duty_cycle = LPC_PWM1->MR2;
            break;
        case PWM_PIN_2_2: /*PWM1.3*/
            duty_cycle = LPC_PWM1->MR3;
            break;
        case PWM_PIN_2_3: /*PWM1.4*/
            duty_cycle = LPC_PWM1->MR4;
            break;
        case PWM_PIN_2_4: /*PWM1.5*/
            duty_cycle = LPC_PWM1->MR5;
            break;
        case PWM_PIN_2_5: /*PWM1.6*/
            duty_cycle = LPC_PWM1->MR6;
            break;
    }
    return duty_cycle;
}

/**
 * How to set PWM Frequency?
 * 1) Convert frequency_Hz to the appropriate match register value
 * 2) Assign the above value to MR0
 * PWM Frequency: 1Hz
 */
void PWMDriver::setFrequency(uint32_t frequency_Hz)
{
    u0_dbg_printf("Inside setFrequency()\n");
    u0_dbg_printf("CPU CLK Freq:%i\n", sys_get_cpu_clock());
    u0_dbg_printf("User set PWM Freq: %i\n", frequency_Hz);
    /*Set Frequency cycle rate of Single Edge PWM via MR0*/
    //Books Socialledge: Registers of Relevance
    /*How to set PWM frequency?*/

    /*1. Set Prescale Counter to 0, it ctrls tick rate of hw counter*/
    LPC_PWM1->PC = 0;

    /*Optional: TC is TC = (CPU Freq)/(PC+1), and counts at rate of*/
    /*LPC_PWM1->TC = ( sys_get_cpu_clock() ) / ( LPC_PWM1->PC + 1 );*/
    /*Note: I will only need to use the above equation to calc Match Reg
    if my PC is > 0, else I can just use sys_get_cpu_clock in my eqn */

    /*2. Convert PWM Freq to appropriate Match Reg*/
    /*MR0 = (CPU CLK Freq)/(PWM Freq)*/
    LPC_PWM1->MR0 = ( sys_get_cpu_clock() )/ ( frequency_Hz );
    LPC_PWM1->LER |= (1 << 0); /*Latch in MR0 value for last value written to PWM MR0*/
    u0_dbg_printf("MR0 reg = %i\n", LPC_PWM1->MR0);
    /*Note: With above eqn, we used Match Reg 0 to set "Max Count".
    MR0 controls the PWM cycle rate, so when TC counts and reaches
    the "Max Count", a match occurs and TC is reset.*/
}

PWMDriver::~PWMDriver()
{
    // TODO Auto-generated destructor stub
}

