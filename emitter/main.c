#include "msp.h"


/**
 * main.c
 */
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	// Configure GPIO for PWM output
	P5->DIR   |=  BIT6;         // P2.4 set TA0.1
	P5->SEL0  |=  BIT6;
	P5->SEL1  &=~ BIT6;

	TIMER_A2->CCR[0] = 26782;            // PWM Period (# cycles of clock)
	TIMER_A2->CCR[1] = 13391;                // CCR1 PWM  DC in 10ths of percent
	//TIMER_A2->CCR[0] =
	//TIMER_A2->CCR[1] =

	TIMER_A2->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;    // CCR1 reset/set mode 7 
	TIMER_A2->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP |TIMER_A_CTL_CLR | TIMER_A_CTL_ID__8;

	while (1)
	{
	    ;
	}
}
