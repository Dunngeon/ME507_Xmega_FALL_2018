//*************************************************************************************
/** @file motorDriver.cpp
 *	  Provides motorDriver classes to perform closed-loop PI control on DC motors
 *    modulated with a motor driver by sending a PWM signal and direction pins.
 *
 *  @b Revisions:
 *    11-26-18 RT Original file
 *	  12-5-18 RT Troubleshooting, added angular PID control
 *
 *  @b Usage:
 *    This file is intended to be used on an XMEGA MCU, providing classes to run motors
 *	  in closed-loop PID control. As of Fall 2018, only PI control is enabled.
 *
 *  @b License:
 *    This file is copyright 2018 by Ricky Tan and released under the Lesser GNU
 *    Public License, version 3. It intended for educational use only, but its use
 *    is not limited thereto, except by the fact that it's not really useful for
 *    anything else, so nobody in his right (or left) mind would try to use it.
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUEN-
 *    TIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 *    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//*************************************************************************************

// Header file for this project
#include "motorDriver.h"

//-------------------------------------------------------------------------------------
/** @brief   Create a motorDriver object.
 *  @details This constructor creates a motorDriver object with the input enable and
 *			 PWM pin locations. The constructor initializes all other variables at zero.
 *  @param   motor The motor that this object corresponds to. Either '1' or '2'.
 */

motorDriver::motorDriver(char motor, emstream* ser_dev)
 {
	 // Setting PWM frequency
	 uint16_t pwm_freq = 500;	// Hardcoded. Edit here if necessary

	 // Serial setup
	 ser_out = ser_dev;			// sets ser_out to debug serial port.
	 motor_ID = motor;

	 // Setting pins
	 if (motor == '1')
	 {
		 pwm_init(&pwm_1, PWM_TCC0, PWM_CH_A, pwm_freq);
		 pwm_init(&pwm_2, PWM_TCC0, PWM_CH_B, pwm_freq);
		 motor_dir = 1;

	 } else if (motor == '2')
	 {
		 pwm_init(&pwm_1, PWM_TCC0, PWM_CH_C, pwm_freq);
		 pwm_init(&pwm_2, PWM_TCC0, PWM_CH_D, pwm_freq);
		 motor_dir = -1;
	 }

	 // Initializing variables. Everything is set to zero or unity to ensure motor does not run.
	 zero_esum_l();
	 zero_esum_a();
	 set_k_l(0,0,0);
	 set_k_a(0,0,0);
	 set_pwm_lim(0);
	 set_pwm_lim_linear(0);
	 set_esum_l_lim(0);
	 set_esum_a_lim(0);
	 set_setpoint_l(0);
	 set_setpoint_a(0);
	 set_position(0);
	 set_angle(0);
	 set_pwm_scaling(1);
   
 }

//-------------------------------------------------------------------------------------
/** @brief   Sets the linear PID gains and the angular gain
 *  @details This method sets the linear PID gains for the object, kp_l, ki_l, kd_l.
 *			 the protected class variables, kp, ki, kd, and ka.
 *  @param   kp_l_in The input linear proportional gain.
 *  @var     kp_l The class variable for the linear proportional gain.
 *  @param   ki_l_in The input linear integral gain.
 *  @var     ki_l The class variable for the linear integral gain.
 *  @param   kd_l_in The input linear proportional gain.
 *  @var     kd_l The class variable for the linear proportional gain.
 */

void motorDriver::set_k_l(int16_t kp_l_in, int16_t ki_l_in, int16_t kd_l_in)
 {
	 // Setting linear PID gains
	 kp_l = kp_l_in;
	 ki_l = ki_l_in;
	 kd_l = kd_l_in;

 }


 //-------------------------------------------------------------------------------------
/** @brief   Sets the angular PID gains and the angular gain
 *  @details This method sets the linear PID gains for the object, kp_l, ki_l, kd_l.
 *  @param   kp_a_in The input angular proportional gain.
 *  @var     kp_a The class variable for the angular proportional gain.
 *  @param   ki_a_in The input angular integral gain.
 *  @var     ki_a The class variable for the angular integral gain.
 *  @param   kd_a_in The input angular proportional gain.
 *  @var     kd_a The class variable for the angular proportional gain.
 */

void motorDriver::set_k_a(int16_t kp_a_in, int16_t ki_a_in, int16_t kd_a_in)
 {
	 // Setting linear PID gains
	 kp_a = kp_a_in;
	 ki_a = ki_a_in;
	 kd_a = kd_a_in;

 }
 

 //-------------------------------------------------------------------------------------
/** @brief   Sets the pwm scaling.
 *  @details This method sets the pwm scaling for each of the gains. This factor divides
 *			 the pwm signal to increase resolution.
 *  @param   pwm_scale_in The input pwm scaling.
 *  @var     pwm_scale The class variable for pwm scaling.
 */

void motorDriver::set_pwm_scaling(int16_t pwm_scale_in)
 {
	 // Setting scalings
	 pwm_scale = pwm_scale_in;
}


//-------------------------------------------------------------------------------------
/** @brief   Sets the limit for the pwm output.
 *  @details This method sets the pwm output limit of the object, the protected
 *		     variable pwm_lim, in pwm output. Make sure a positive number is entered.
 *  @param   pwm_lim_in The input pwm limit.
 *  @var     pwm_lim The class variable for the pwm limit.
 */

void motorDriver::set_pwm_lim(int16_t pwm_lim_in)
 {
	 // Setting pwm_lim
	 if (pwm_lim_in > 100)
	 {
		 pwm_lim = 100;
	 } else 
	 {
		 pwm_lim = pwm_lim_in;
	 }
	 
 }


 //-------------------------------------------------------------------------------------
/** @brief   Sets the limit for the linear pwm output.
 *  @details This method sets the linear pwm output limit of the object. The difference
 *			 between pwm_lim and pwm_lim_linear is the space with which the angular PID
 *			 operates. Input is limited to pwm_lim, so make sure to set pwm_lim first.
 *  @param   pwm_lim_linear_in The input pwm limit.
 *  @var     pwm_lim_linear The class variable for the pwm limit.
 */

void motorDriver::set_pwm_lim_linear(int16_t pwm_lim_linear_in)
 {
	 // Setting pwm_lim
	 if (pwm_lim_linear_in > pwm_lim)
	 {
		 pwm_lim_linear = pwm_lim;
	 } else 
	 {
		 pwm_lim_linear = pwm_lim_linear_in;
	 }
	 //pwm_lim_linear = pwm_lim_linear_in;
	 
 }


 //-------------------------------------------------------------------------------------
/** @brief   Sets the limit for the linear integral error sum.
 *  @details This method sets the integral error sum limit of the object, the protected
 *		     variable esum_l_lim, in encoder counts.
 *  @param   esum_l_lim_in The input integral error sum limit.
 *  @var     esum_l_lim The class variable for the integral error sum limit.
 */

void motorDriver::set_esum_l_lim(int16_t esum_l_lim_in)
 {
	 // Setting esum_l_lim
	 esum_l_lim = esum_l_lim_in;
 }


 //-------------------------------------------------------------------------------------
/** @brief   Sets the limit for the angular integral error sum.
 *  @details This method sets the integral error sum limit of the object, the protected
 *		     variable esum_a_lim, in encoder counts.
 *  @param   esum_a_lim_in The input integral error sum limit.
 *  @var     esum_a_lim The class variable for the integral error sum limit.
 */

void motorDriver::set_esum_a_lim(int16_t esum_a_lim_in)
 {
	 // Setting esum_a_lim
	 esum_a_lim = esum_a_lim_in;
 }


 //-------------------------------------------------------------------------------------
/** @brief   Sets the target linear setpoint.
 *  @details This method sets the target linear setpoint of the object, the protected
 *		     variable setpoint_l, in encoder counts.
 *  @param   setpoint_l_in The input linear setpoint.
 *  @var     setpoint_l The class variable for the linear setpoint.
 */

void motorDriver::set_setpoint_l(int16_t setpoint_l_in)
 {
	 // Setting setpoint_l
	 setpoint_l = setpoint_l_in;
 }


  //-------------------------------------------------------------------------------------
/** @brief   Sets the target angular setpoint.
 *  @details This method sets the target angular setpoint of the object, the protected
 *		     variable setpoint_a, in encoder counts.
 *  @param   setpoint_a_in The input angular setpoint.
 *  @var     setpoint_a The class variable for the angular setpoint.
 */

void motorDriver::set_setpoint_a(int16_t setpoint_a_in)
 {
	 // Setting setpoint_a
	 setpoint_a = setpoint_a_in;
 }


//-------------------------------------------------------------------------------------
/** @brief   Sets the current position.
 *  @details This method sets the current position of the object, the protected
 *		     variable position, in encoder counts.
 *  @param   position_in The input position.
 *  @var     position The class variable for the position.
 */

void motorDriver::set_position(int16_t position_in)
 {
	 // Setting position
	 position = position_in;
 }


//-------------------------------------------------------------------------------------
/** @brief   Sets the current angle.
 *  @details This method sets the current angle of the object, the protected
 *		     variable angle.
 *  @param   angle_in The input angle.
 *  @var     angle The class variable for the angle.
 */

void motorDriver::set_angle(int16_t angle_in)
 {
	 // Setting angle
	 angle = angle_in;
 }


//-------------------------------------------------------------------------------------
/** @brief   Resets the linear integral error sum to zero.
 *  @details This method resets the integral linear error sum, the protected
 *		     esum_l, to zero.
 *  @param   esum_l The sum of the linear error terms for integral gain in encoder counts.
 */

void motorDriver::zero_esum_l(void)
 {
	 // Resetting esum_l
	 esum_l = 0;
 }


//-------------------------------------------------------------------------------------
/** @brief   Resets the angular integral error sum to zero.
 *  @details This method resets the angular integral error sum, the protected
 *		     esum_a, to zero.
 *  @param   esum_a The sum of the linear error terms for angular gain.
 */

void motorDriver::zero_esum_a(void)
 {
	 // Resetting esum_a
	 esum_a = 0;
 }


//-------------------------------------------------------------------------------------
/** @brief   Calculates the current pwm output signal.
 *  @details This method calculates the current pwm output signal. Its arguments are booleans
 *			 dictating whether to include proportional, integral, and derivative action.
 *			 This method sums the difference between setpoint and position to get the error as
 *           well as incrementing the error sum, esum, by the error. It does NOT update the
 *           position or setpoint variable of the motorDriver object. The drive can be either
 *			 drive-coast (op_type=0) or drive-brake (op_type=1). Coast is lower performance
 *			 and lower power, brake is higher performance higher power.
 *  @param   en_p Enable proportional action input.
 *  @param   en_i Enable integral action input.
 *  @param   en_d Enable derivative action input.
 *	@param	 op_type Whether operation is drive-coast or drive-brake
 */



diagnostic motorDriver::run(bool en_p = 0, bool en_i = 0, bool en_d = 0, bool op_type = 0)
 {
	// Calculating error terms
   error_l = setpoint_l - position;
   error_a = setpoint_a - angle;
   esum_l += error_l;
   if (esum_l > esum_l_lim) // Rounding esum_l to esum_l_lim on either end
   {
       esum_l = esum_l_lim;
   } else if (esum_l < -esum_l_lim)
   {
       esum_l = -esum_l_lim;
   }
   esum_a += error_a;
   if (esum_a > esum_a_lim) // Rounding esum_a to esum_a_lim on either end
   {
	   esum_a = esum_a_lim;
   } else if (esum_a < -esum_a_lim)
   {
	   esum_a = -esum_a_lim;
   }
   eder_l = 0;  // Placeholder
   eder_a = 0;  // Placeholder

   // Calculating linear signal components
   int16_t signal_l_p = kp_l*error_l;
   int16_t signal_l_i = ki_l*esum_l;
   int16_t signal_l_d = kd_l*eder_l;
   int16_t pwm_percent_linear = (((int16_t) en_p)*signal_l_p + ((int16_t) en_i)*signal_l_i + ((int16_t) en_d)*signal_l_d)/pwm_scale;
   if (pwm_percent_linear > pwm_lim_linear)	// Saturating linear component of pwm_percent
   {
	   pwm_percent_linear = pwm_lim_linear;
   }
   else if (pwm_percent_linear < -pwm_lim_linear)
   {
	   pwm_percent_linear = -pwm_lim_linear;
   } 

   // Calculating angular signal components
   int16_t signal_a_p = kp_a*error_a;
   int16_t signal_a_i = ki_a*esum_a;
   int16_t signal_a_d = kd_a*eder_a;
   int16_t pwm_percent_angular = (((int16_t) en_p)*signal_a_p + ((int16_t) en_i)*signal_a_i + ((int16_t) en_d)*signal_a_d)*motor_dir/pwm_scale;	// This entire term needs to be either positive or negative depending on how we define the angle externally
   if (pwm_percent_angular > (pwm_lim - pwm_lim_linear))	// Saturating angular component of pwm_percent
   {
	   pwm_percent_angular = pwm_lim - pwm_lim_linear;
   }
   else if (pwm_percent_angular < -(pwm_lim - pwm_lim_linear))
   {
	   pwm_percent_angular = -(pwm_lim - pwm_lim_linear);
   }
   
   // Calculating total signal to motors
   int16_t pwm_percent = (pwm_percent_linear + pwm_percent_angular)*motor_dir;
   
   // Serial diagnostics
   //*ser_out << "Motor " << motor_ID << " Signal P " << signal_l_p << " I " << signal_l_i << " esum " << esum_l << " D " << signal_l_d << " PWM % " << pwm_percent << endl;
   
   // Limiting total pwm percentage 
   if (pwm_percent > pwm_lim)	
   {
	   pwm_percent = pwm_lim;
   } 
   else if (pwm_percent < -pwm_lim)
   {
	   pwm_percent = -pwm_lim;
   }

   // Operating motor
   if (op_type)	// Drive-brake mode. Modulates brake pin instead of drive pin, higher performance (?).
   {
	   if (pwm_percent > 0)
	   {
		   pwm_start(&pwm_1, 100);	// Setting IN1 high 
		   pwm_start(&pwm_2, 100 - abs(pwm_percent));	// Modulating IN2 low
	   } else {
		   pwm_start(&pwm_1, 100 - abs(pwm_percent));	// Modulating IN1 low
		   pwm_start(&pwm_2, 100);	// Setting IN2 high
	   }
   } 
   else // Drive-coast mode. Modulates drive pin, higher current decay
   { 
	   if (pwm_percent > 0)
	   {
		   pwm_start(&pwm_1, abs(pwm_percent));	// Modulating IN1 high 
		   pwm_start(&pwm_2, 0);	// Setting IN2 low
	   } else {
		   pwm_start(&pwm_1, 0);	// Setting IN1 low
		   pwm_start(&pwm_2, abs(pwm_percent));	// Modulating IN2 high
	   }
   }

   diagnostic diag;
   diag.pwm_tot = pwm_percent;
   diag.pwm_lin = pwm_percent_linear;
   diag.pwm_ang = pwm_percent_angular;
   diag.esum_a_ = esum_a;
   diag.esum_l_ = esum_l;

   return diag;
 }


//-------------------------------------------------------------------------------------
// TEST CODE

/*int main(void)
{
	// Initializing objects driver
	char m1 = '1';
	char m2 = '2';
	motorDriver motor1 (m1);
	motorDriver motor2 (m2);

	// Setting proportional gains
	uint16_t kp = 1000;
	motor1.set_kp(kp);
	motor2.set_kp(kp);

	// Setting limits
	motor1.set_pwm_lim(100);
	motor2.set_pwm_lim(100);
	motor1.set_esum_lim(10);
	motor2.set_esum_lim(10);

	// Setting test condition error
	int16_t err = 50;
	int16_t setpoint = 100;
	int16_t position = setpoint - err;
	motor1.set_position(position);
	motor2.set_position(position);
	motor1.set_setpoint(setpoint);
	motor2.set_setpoint(setpoint);
 
	while(1)
	{
		// Running motors
		motor1.run(true,false,false,false);
		motor2.run(true,false,false,true);
	}

}
*/