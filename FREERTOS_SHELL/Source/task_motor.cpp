//**************************************************************************************
/** \file task_motor.cpp
 *    This file contains source code for a motor controller task.
 *
 *  Revisions:
 *    \li 12-4-18 RT Original file
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

#include "task_motor.h"

// Shares
int16_t pwm_tot_1 = 0;
int16_t pwm_tot_2 = 0;
int16_t esum_l_1 = 0;
int16_t esum_l_2 = 0;
int16_t esum_a_1 = 0;
int16_t esum_a_2 = 0;
int16_t pwm_lin_1 = 0;
int16_t pwm_lin_2 = 0;
int16_t pwm_ang_1 = 0;
int16_t pwm_ang_2 = 0;
int16_t LinearDistance;			// Current linear distance
int16_t setpoint_a_1 = 0;		// Contains current angular setpoint of Robot

//-------------------------------------------------------------------------------------
/** @brief   Constructor for task_motor. Utilizes base task frt_task
 *  @details This constructor creates a new motor task. Its main job is to call the
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
/** @brief   Run task for task_motor
 *  @details This task initializes the motor objects and updates the motors with their 
 *		current position and setpoints (intertask variables) and adjusts the output pwm signals.
 *		It also updates shared task variables for task_diag for diagnostic use.
*   @var motorDriver motor1 Object of class motorDriver for first motor
*   @var motorDriver motor2 Object of class motorDriver for second motor
*   @var int16_t pwm_scale Scaling factor for PWM output. Divides summed signals to allow for
*			greater resolution.
*   @var int16_t kp_l Linear proportional gain. Given to both motors
*   @var int16_t ki_l Linear integral gain. Given to both motors
*   @var int16_t kd_l Linear derivative gain. Given to both motors
*   @var int16_t kp_a Angular proportional gain. Given to both motors
*   @var int16_t ki_a Angular integral gain. Given to both motors
*   @var int16_t kd_a Angular derivative gain. Given to both motors
*   @var int16_t pwm_lim Limit for the total PWM output
*   @var int16_t pwm_lim_linear Limit for the proportion of the total PWM signal allowed for
*			the linear control loop. pwm_lim_linear - pwm_lin is the portion for angular
*   @var int16_t esum_l_lim Limit for the accumulation of linear error terms
*   @var int16_t esum_a_lim Limit for the accumulation of angular error terms
*   @var diagnostic diag_1 Object of struct diagnostic for motor 1 that catches the output of the motorDriver
*			run method. Its values are passed to shared task variables used by task_diag
*			to print diagnostic information
*   @var diagnostic diag_2 Same as diag_1 but for motor 2
*   @var int16_t LinearDistance Shared task variable containing the linear target distance. Drives the
*			linear control loop
*   @var int16_t AngleGoal Shared task variable for target angle for robot
*   @var int16_t Robot_Angle_Theta_INERT Shared task variable for current robot angle
*   @var int16_t setpoint_a_1 Angular setpoint given to both motors. Development artefact
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
	int16_t pwm_scale = 100;
	motor1.set_pwm_scaling(pwm_scale);
	motor2.set_pwm_scaling(pwm_scale);

	// Setting linear gains
	int16_t kp_l = 30;
	int16_t ki_l = 1;
	int16_t kd_l = 0;
	motor1.set_k_l(kp_l,ki_l,kd_l);
	motor2.set_k_l(kp_l,ki_l,kd_l);

	// Setting angular gains
	int16_t kp_a = 60;
	int16_t ki_a = 10;
	int16_t kd_a = 0;
	motor1.set_k_a(kp_a,ki_a,kd_a);
	motor2.set_k_a(kp_a,ki_a,kd_a);
	
	// Setting limits
		// The limit for pwm_lim is 100. The purpose of pwm_lim is to artificially
		// limit the output of the motors. Setting to 100 == no artificial cap.
		// esum_lim is the limit for the accumulation of errors for integral gain.
	int16_t pwm_lim = 50;	// Max percentage pwm output
	int16_t pwm_lim_linear = 3*pwm_lim/5;	// Portion of pwm_lim used for linear driving. Rest is for angular
	int16_t esum_l_lim = pwm_lim*pwm_scale/10;	// Note: esum_lim is used prior to the pwm being scaled
	int16_t esum_a_lim = (pwm_lim-pwm_lim_linear)*pwm_scale;
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

	motor1.set_setpoint_l(100);
	motor2.set_setpoint_l(100);

	motor1.set_setpoint_a(100);
	motor2.set_setpoint_a(100);
	//------------------------------*/

	// Diagnostic outputs
	diagnostic diag_1;
	diagnostic diag_2;

	// This is an infinite loop; it runs until the power is turned off. This loop
	// continually updates motor position, setpoint, and pwm output.
	portTickType previousTicks = xTaskGetTickCount ();
	*p_serial << "this should only appear once" << endl;
	while(1)
	{
		
		//calculate linear distance from the setpoint, then pass that linear distance to both motors. We control distance from goal and angular heading.
		LinearDistance = sqrt(pow((setpoint_l_2 - Robot_Pos_Y_INERT),2) + pow((setpoint_l_1-Robot_Pos_X_INERT),2));
		int16_t AngleGoal = atan2((setpoint_l_2 - Robot_Pos_Y_INERT), (setpoint_l_1-Robot_Pos_X_INERT));
		setpoint_a_1 = AngleGoal - LinearDistance;
		if(setpoint_l_1-Robot_Pos_X_INERT <= 0)
		{
			LinearDistance = LinearDistance * -1;
		}
		else {
			//do nothing
		}
		
		// Updating positions, always zero because we've calculated the error above
		motor1.set_position(0);
		motor2.set_position(0);
		
		// Updating motor linear setpoints
		motor1.set_setpoint_l(LinearDistance);
		motor2.set_setpoint_l(LinearDistance);

		// Updating angles
		motor1.set_angle(AngleGoal - Robot_Angle_Theta_INERT);
		motor2.set_angle(AngleGoal - Robot_Angle_Theta_INERT);
		
		// Updating angular setpoints, setpoint_a_1 is the goal angle of the robot.
		motor1.set_setpoint_a(setpoint_a_1);
		motor2.set_setpoint_a(setpoint_a_1);

		// Updating pwm outputs
			// Input is run(proportional, integral, derivative, mode).
			// PWM signal is calculated using the sum of each action that is
			// enabled (the first three arguments). The pins are modulated
			// according to the drive mode. True means higher performance,
			// false means lower current (ish). Higher performance is recommended,
			// especially for lower pwm's.
		diag_1 = motor1.run(true,true,false,true);
		diag_2 = motor2.run(true,true,false,true);	// Currently is set to run in PI mode

		// Updating diagnostic shares
		pwm_tot_1 = diag_1.pwm_tot;
		pwm_tot_2 = diag_2.pwm_tot;
		pwm_lin_1 = diag_1.pwm_lin;
		pwm_lin_2 = diag_2.pwm_lin;
		pwm_ang_1 = diag_1.pwm_ang;
		pwm_ang_2 = diag_2.pwm_ang;
		esum_l_1 = diag_1.esum_l_;
		esum_l_2 = diag_2.esum_l_;
		esum_a_1 = diag_1.esum_a_;
		esum_a_2 = diag_2.esum_a_;

		// Sending serial diagnostics
		//*p_serial << "PWM setting (M1)" << pwm_1 << " (M2) " << pwm_2 << " time " << previousTicks << endl;

		// Delaying
		previousTicks = xTaskGetTickCount();
		delay_from_to_ms(previousTicks,10);
	}

}
