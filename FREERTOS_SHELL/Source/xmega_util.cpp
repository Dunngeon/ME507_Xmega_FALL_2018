/*
 * xmega_util.c
 *
 * Created: 11/11/2017 1:20:28 PM
 *  Author: crefvem
 */ 
#include "xmega_util.h"

/*! \brief CCP write helper function written in assembly.
 *
 *  This function is written in assembly because of the time critical
 *  operation of writing to the registers.
 *
 *  \param address A pointer to the address to write to.
 *  \param value   The value to put in to the register.
 */
void CCPWrite( volatile uint8_t * address, uint8_t value )
{
	#if defined __GNUC__
	uint8_t volatile saved_sreg = SREG;
	cli();
	volatile uint8_t * tmpAddr = address;
	#ifdef RAMPZ
	RAMPZ = 0;
	#endif
	asm volatile(
	"movw r30,  %0"	      "\n\t"
	"ldi  r16,  %2"	      "\n\t"
	"out   %3, r16"	      "\n\t"
	"st     Z,  %1"       "\n\t"
	:
	: "r" (tmpAddr), "r" (value), "M" (0xD8), "i" (&CCP)
	: "r16", "r30", "r31"
	);

	SREG = saved_sreg;
	#endif
}


// Configure the system clock
void config_SYSCLOCK()
{
	// this function is condensed AVR1003 example code that only enables a 16MHz external crystal -> 32Mhz clock or use of internal crystal.
	uint8_t volatile saved_sreg = SREG;
	cli();
	
	//below code is for enabling external 16MHz crystal to 32MHz clock on XMega MCU
	OSC.XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_XTAL_16KCLK_gc;				// Configure the external clock frequency and configure startup time
	OSC.CTRL |= OSC_XOSCEN_bm;														// Enable the external clock
	do {} while((OSC.STATUS & (OSC_XOSCRDY_bm)) != (OSC_XOSCRDY_bm));					// Wait for a stable clock
	OSC.PLLCTRL = OSC_PLLSRC_XOSC_gc | OSC_PLLFAC1_bm;								// Configure the PLL factor and divider and select external clock as source
	OSC.CTRL |= OSC_PLLEN_bm;															// Enable the PLL
	do {} while((OSC.STATUS & (OSC_PLLRDY_bm)) != (OSC_PLLRDY_bm));					// Wait for a stable PLL clock
	
	//This code below is CLKSYS_Main_ClockSource_Select from AVR1003 ex code for 16MHz -> 32MHz clock PLL
	uint8_t CLK_CTRL_TEMP = (CLK.CTRL & ~CLK_SCLKSEL_gm) | (CLK_SCLKSEL_PLL_gc);
	CLK.CTRL = CLK_CTRL_TEMP;
	CCP = 0xD8;																		//this should be already contained in CCPWrite, so redundant I think
	CCPWrite(&(CLK.CTRL),CLK_CTRL_TEMP);
	OSC.CTRL &= ~(CLK_SCLKSEL_PLL_gc);
	SREG = saved_sreg;
	
	////all below is for configuring internal clock
	//OSC.CTRL |= OSC_RC32MEN_bm;														// Enable the internal clock
	//do {} while((OSC.STATUS & (OSC_RC32MRDY_bm)) != (OSC_RC32MRDY_bm));				// Wait for a stable clock
	//
	////This code below is CLKSYS_Main_ClockSource_Select from AVR1003 ex code
	//uint8_t CLK_CTRL_TEMP = (CLK.CTRL & ~CLK_SCLKSEL_gm) | (CLK_SCLKSEL_RC32M_gc);
	//CLK.CTRL = CLK_CTRL_TEMP;
	//CCP = 0xD8;																		//this should be already contained in CCPWrite, so redundant I think
	//CCPWrite(&(CLK.CTRL),CLK_CTRL_TEMP);
	//OSC.CTRL &= ~(OSC_RC2MEN_bm);
	//SREG = saved_sreg;
}