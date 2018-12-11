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

#include "task_motor.h"

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

task_motor::task_motor (const char* a_name, 
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
/** This task updates the motors with their current position and setpoints (intertask
*	variables) and adjusts the output pwm signals.
*/

void task_motor::run (void)
{
	// Initializing objects driver
		// All gains, limits, and positions are initialized to zero.
	motorDriver motor1 ('1', p_serial);
	motorDriver motor2 ('2', p_serial);
	
	// Setting gain scaling
		// Gain scaling is the factor by which signals are divided. Allows for gains
		// to be scaled appropriately for int limits. Currently set to 1 (no scaling).
	int16_t pwm_scale = 10;
	motor1.set_pwm_scaling(pwm_scale);
	motor2.set_pwm_scaling(pwm_scale);

	// Setting linear gains
	int16_t kp_l = 1;
	int16_t ki_l = 1;
	int16_t kd_l = 0;
	motor1.set_k_l(kp_l,ki_l,kd_l);
	motor2.set_k_l(kp_l,ki_l,kd_l);

	// Setting angular gains
	int16_t kp_a = 1;
	int16_t ki_a = 1;
	int16_t kd_a = 0;
	motor1.set_k_a(kp_a,ki_a,kd_a);
	motor2.set_k_a(kp_a,ki_a,kd_a);
	
	// Setting limits
		// The limit for pwm_lim is 100. The purpose of pwm_lim is to artificially
		// limit the output of the motors. Setting to 100 == no artificial cap.
		// esum_lim is the limit for the accumulation of errors for integral gain.
	int16_t pwm_lim = 100;	// Max percentage pwm output
	int16_t pwm_lim_linear = 80;	// Portion of pwm_lim used for linear driving. Rest is for angular
	int16_t esum_l_lim = 80*pwm_scale;	// Note: esum_lim is used prior to the pwm being scaled
	int16_t esum_a_lim = 20*pwm_scale;
	motor1.set_pwm_lim(pwm_lim); // Needs to be performed prior to set_pwm_lim_linear
	motor2.set_pwm_lim(pwm_lim);
	motor1.set_pwm_lim_linear(pwm_lim_linear);
	motor2.set_pwm_lim_linear(pwm_lim_linear);
	motor1.set_esum_l_lim(esum_l_lim);
	motor2.set_esum_l_lim(esum_l_lim);
	motor1.set_esum_a_lim(esum_a_lim);
	motor2.set_esum_a_lim(esum_a_lim);


	/*//-------------------------------
	// Test condition
	motor1.set_position(0);	
	motor2.set_position(0);

	motor1.set_angle(0);
	motor2.set_angle(0);

	motor1.set_setpoint_l(3);
	motor2.set_setpoint_l(3);

	motor1.set_setpoint_a(3);
	motor2.set_setpoint_a(3);
	//------------------------------*/

	// PWM percent outputs
	int16_t pwm_1;
	int16_t pwm_2;

	// This is an infinite loop; it runs until the power is turned off. This loop
	// continually updates motor position, setpoint, and pwm output.
	portTickType previousTicks = xTaskGetTickCount ();
	*p_serial << "this should only appear once" << endl;
	while(1)
	{
		// Updating positions
		motor1.set_position(position_1);
		motor2.set_position(position_2);
		
		// Updating linear setpoints
		motor1.set_setpoint_l(setpoint_l_1);
		motor2.set_setpoint_l(setpoint_l_2);

		// Updating angles
		motor1.set_angle(position_1);
		motor2.set_angle(position_2);
		
		// Updating angular setpoints
		motor1.set_setpoint_a(setpoint_l_1);
		motor2.set_setpoint_a(setpoint_l_2);

		// Updating pwm outputs
			// Input is run(proportional, integral, derivative, mode).
			// PWM signal is calculated using the sum of each action that is
			// enabled (the first three arguments). The pins are modulated
			// according to the drive mode. True means higher performance,
			// false means lower current (ish). Higher performance is recommended,
			// especially for lower pwm's.
		pwm_1 = motor1.run(true,false,false,true);
		pwm_2 = motor2.run(true,false,false,true);	// Currently is set to run in PI mode

		// Sending serial diagnostics
		*p_serial << "PWM setting (M1)" << pwm_1 << " (M2) " << pwm_2 << " time " << previousTicks << endl;

		// Delaying
		previousTicks = xTaskGetTickCount();
		delay_from_to_ms(previousTicks,5);
	}

}
