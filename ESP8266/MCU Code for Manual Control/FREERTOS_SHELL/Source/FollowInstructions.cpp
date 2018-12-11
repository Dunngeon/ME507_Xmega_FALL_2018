/*
 * FollowInstructions.cpp
 *
 * Created: 12/6/2018 12:29:06 AM
 *  Author: Alec
 */ 

//**************************************************************************************
/** \file task_motor.cpp
 *    This file contains source code for a motor controller task.
 *
 *  Revisions:
 *    12-4-18 RT Original file
 *
 *  License:
 *    This file is copyright 2012 by Ricky Tan and released under the Lesser GNU 
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

#include "FollowInstructions.h"

//KEEP THESE AS GLOBALS :(
int16_t position_1 = 0;			// Contains current position of motor 1
int16_t position_2 = 0;			// Contains current position of motor 2
int16_t setpoint_l_1 = 0;		// Contains current linear setpoint of motor 1
int16_t setpoint_l_2 = 0;		// Contains current linear setpoint of motor 2
int16_t angle_1 = 0;			// Contains current angle of motor 1
int16_t angle_2 = 0;			// Contains current angle of motor 2
int16_t setpoint_a_1 = 0;		// Contains current angular setpoint of motor 1
int16_t setpoint_a_2 = 0;		// Contains current angular setpoint of motor 2


FollowInstructions::FollowInstructions (const char* a_name,
unsigned portBASE_TYPE a_priority,
size_t a_stack_size,
emstream* p_ser_dev
)
: frt_task (a_name, a_priority, a_stack_size, p_ser_dev)
{
	
	// Nothing is done in the body of this constructor. All the work is done in the
	// call to the frt_task constructor on the line just above this one
}
	


void FollowInstructions::run (){
	
	int val = 0;
	this->curChar = '\0';
	
	
	while (1){
			
			if (p_serial->check_for_char ()){
				
				this->prevChar = this->curChar;
				this->curChar = p_serial->getchar();
				
					if(this->prevChar == 's'){
						curSpeed = 48-this->curChar;
						setpoint_l_1 = curSpeed*100;
						setpoint_l_2 = curSpeed*200;
					}
					
					if(this->prevChar == 'x'){
						
					}
			}
		
			// Delaying (don't touch this)
			portTickType previousTicks = xTaskGetTickCount();
			delay_from_to_ms(previousTicks,5);
	}

}
