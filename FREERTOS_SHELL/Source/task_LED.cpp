//**************************************************************************************
/** \file task_LED.cpp
 *    This file contains source code for a user interface task for a ME405/FreeRTOS
 *    test suite. 
 *
 *  Revisions:
 *    \li 09-16-2018 CTR Adapted from JRR task_user.cpp
 *    \li 09-30-2012 JRR Original file was a one-file demonstration with two tasks
 *    \li 10-05-2012 JRR Split into multiple files, one for each task
 *    \li 10-25-2012 JRR Changed to a more fully C++ version with class task_user
 *    \li 11-04-2012 JRR Modified from the data acquisition example to the test suite
 *
 *  License:
 *    This file is copyright 2012 by JR Ridgely and released under the Lesser GNU 
 *    Public License, version 2. It intended for educational use only, but its use
 *    is not limited thereto. */
/*    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 *    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUEN-
 *    TIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 *    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 *    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 *    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//**************************************************************************************

#include <avr/io.h>                         // Port I/O for SFR's
#include <avr/wdt.h>                        // Watchdog timer header

#include "shared_data_sender.h"
#include "shared_data_receiver.h"
#include "task_LED.h"                      // Header for this file


//-------------------------------------------------------------------------------------
/** This constructor creates a new data acquisition task. Its main job is to call the
 *  parent class's constructor which does most of the work.
 *  @param a_name A character string which will be the name of this task
 *  @param a_priority The priority at which this task will initially run (default: 0)
 *  @param a_stack_size The size of this task's stack in bytes 
 *                      (default: configMINIMAL_STACK_SIZE)
 *  @param p_ser_dev Pointer to a serial device (port, radio, SD card, etc.) which can
 *                   be used by this task to communicate (default: NULL)
 */

task_LED::task_LED (const char* a_name, 
					  unsigned portBASE_TYPE a_priority, 
					  size_t a_stack_size,
					  emstream* p_ser_dev
					 )
	: frt_task (a_name, a_priority, a_stack_size, p_ser_dev)
{
	// Nothing is done in the body of this constructor. All the work is done in the
	// call to the frt_task constructor on the line just above this one
}


//-------------------------------------------------------------------------------------
/** This task blinks an LED attached to PORTR Pin 1
 */

void task_LED::run (void)
{
	// Make a variable which will hold times to use for precise task scheduling
	portTickType previousTicks = xTaskGetTickCount ();

	// Wait a little while for user interface task to finish up
	delay_ms(10);


	while(1)
	{
		switch (state)
		{
		case INIT:
			ctr=0;
			PORTR.OUTCLR = PIN0_bm;				// Make sure the pin is off before configuring it as output
			PORTR.DIRSET = PIN0_bm;				// Set the pin as an output
			transition_to(LED_SET);
			break;
			
		case LED_SET:
			PORTR.OUTSET = PIN0_bm;
			transition_to(LED_ON);
			break;
			
		case LED_ON:
			PORTR.OUTCLR = PIN0_bm;
			if(++ctr==10)
			{
				ctr = 0;
				transition_to(LED_CLR);
			}
			break;
			
		case LED_CLR:
			PORTR.OUTSET = PIN0_bm;
			transition_to(LED_OFF);
			break;
			
		case LED_OFF:
			if(++ctr==90)
			{
				ctr = 0;
				transition_to(LED_SET);
			}
			break;
		
		default:
			break;
		}
		runs++;
		delay_from_to_ms(previousTicks,10);
	}
}