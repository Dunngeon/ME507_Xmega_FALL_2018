//**************************************************************************************
/** \file task_diag.cpp
 *    This file contains source code for a diagnostic serial print task.
 *
 *  Revisions:
 *    12-5-18 RT Original file
 *
 *  License:
 *    This file is copyright 2018 by Ricky Tan and released under the Lesser GNU 
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

 #include "task_diag.h"

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

task_diag::task_diag (const char* a_name, 
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
/** This task pulls information from shared variables and prints to serial.
*/

void task_diag::run (void)
{
	portTickType previousTicks = xTaskGetTickCount ();

	while(1)
	{
		// Sending serial diagnostics
		*p_serial << "--- MOTOR 1 ---" << endl;
		*p_serial << "| Total PWM: " << pwm_tot_1 << " | Linear PWM: " << pwm_lin_1 << " | Angular PWM: " << pwm_ang_1 << endl;
		*p_serial << "| Robot Position: " << Robot_Pos_X_INERT << " " << Robot_Pos_Y_INERT << " | Angle: " << Robot_Angle_Theta_INERT << endl;
		*p_serial << "| esum-L: " << esum_l_1 << " | esum-A:" << esum_a_1 << endl;
		*p_serial << "| Linear Distance: " << LinearDistance << endl;
		*p_serial << "--- MOTOR 2 ---" << endl;
		*p_serial << "| Total PWM: " << pwm_tot_2 << " | Linear PWM: " << pwm_lin_2 << " | Angular PWM: " << pwm_ang_2 << endl;
		*p_serial << " | Goal Position: " << setpoint_l_1 << " "<< setpoint_l_2 << " | Angle Setpoint: " << setpoint_a_1 << endl;
		*p_serial << "| esum-L: " << esum_l_2 << " | esum-A:" << esum_a_2 << endl;
		*p_serial << "=============================================================";

		// Delaying
		previousTicks = xTaskGetTickCount();
		delay_from_to_ms(previousTicks,750);
	}
	
	}
