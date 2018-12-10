//*************************************************************************************
/*  File:  motorDriver.h
 *	  Provides motorDriver classes to perform closed-loop PI control on DC motors
 *    modulated with a motor driver by sending a PWM signal and direction pins.
 *
 *  Revisions:
 *    11-26-18 RT Original file
 *	  12-5-18 RT Troubleshooting, added angular PID control
 *
 *  Usage:
 *    This file is intended to be used on an XMEGA MCU, providing classes to run motors
 *	  in closed-loop PID control. As of Fall 2018, only PI control is enabled.
 *
 *  License:
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

#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

// Atmel header file
#include <asf.h>
#include <stdlib.h>                         // Prototype declarations for I/O functions

#include "FreeRTOS.h"                       // Primary header for FreeRTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // FreeRTOS inter-task communication queues

#include "rs232int.h"                       // ME405/507 library for serial comm.
#include "time_stamp.h"                     // Class to implement a microsecond timer
#include "frt_task.h"                       // Header for ME405/507 base task class
#include "frt_queue.h"                      // Header of wrapper for FreeRTOS queues
#include "frt_text_queue.h"                 // Header for a "<<" queue class
#include "frt_shared_data.h"                // Header for thread-safe shared data

#include "shares.h"                         // Global ('extern') queue declarations

#include "math.h"

struct diagnostic		// Contains diagnostic information
{
	int16_t pwm_tot;
	int16_t pwm_lin;
	int16_t pwm_ang;
	int16_t esum_a_;
	int16_t esum_l_;
};

//-------------------------------------------------------------------------------------
/** @brief   A PID closed-loop motor controller class.
 *  @details This class allows for closed-loop PI control of a DC motor through a
 *           motor driver chip. Class needs to be given pins for motor.
 */
class motorDriver
{
	protected:
		struct pwm_config pwm_1;	// PWM object for IN1
		struct pwm_config pwm_2;	// PWM object for IN2
		
		emstream *ser_out;  //debugging serial connection
		char motor_ID;		//used for printing motor ID on serial
		int16_t kp_l;			// Linear proportional gain constant
		int16_t ki_l;			// Linear integral gain constant
		int16_t kd_l;			// Linear derivative gain constant
		int16_t kp_a;			// Angular proportional gain constant
		int16_t ki_a;			// Angular integral gain constant
		int16_t kd_a;			// Angular derivative gain constant
		int16_t pwm_scale;	// Scaling for pwm

		int16_t setpoint_l;		// Linear setpoint of motor
		int16_t position;		// Current position of motor in encoder counts
		int16_t angle;			// Current angle of motor
		int16_t setpoint_a;		// Angular setpoint for motor
		
		int16_t motor_dir;		// Direction of motor. 1 or -1 depending on motor		

		int16_t error_l;		// Positional error in encoder counts
		int16_t	esum_l;			// Error sum for integral control
		int16_t eder_l;			// Derivative of the linear error
		int16_t error_a;		// Angular error, units TBD
		int16_t esum_a;			// Angular error integral
		int16_t eder_a;			// Derivative of the angular error

		int16_t pwm_lim;		// Total PWM percentage limit
		int16_t pwm_lim_linear;	// Non-turning PWM percentage limit, pwm_lin_linear < pwm_lim
		int16_t	esum_l_lim;		// Limit for accumulation of error terms
		int16_t	esum_a_lim;		// Limit for accumulation of error terms

		
		
	public:
		motorDriver(char, emstream* ser_dev);	// Default constructor. Give motor '1' or '2'.

		void set_k_l(int16_t kp_l_in, int16_t ki_l_in, int16_t kd_l_in);	// Sets the linear PID gains
		void set_k_a(int16_t kp_a_in, int16_t ki_a_in, int16_t kd_a_in);	// Sets the angular PID gains
		void set_pwm_scaling(int16_t pwm_scale_in); // Sets the pwm scaling
		void set_pwm_lim(int16_t pwm_lim_in);	// Sets the PWM limit, pwm_lim
		void set_pwm_lim_linear(int16_t pwm_lim_linear_in);	// Sets the linear PWM limit, pwm_lim_linear
		void set_esum_l_lim(int16_t esum_l_lim_in);	// Sets the linear error sum limit, esum_l_lim
		void set_esum_a_lim(int16_t esum_a_lim_in);	// Sets the angular error sum limit, esum_a_lim

		void set_setpoint_l(int16_t setpoint_l_in);	// Sets the current linear setpoint for the motor
		void set_setpoint_a(int16_t setpoint_l_in);	// Sets the current angular setpoint for the motor
		void set_position(int16_t position_in);	// Sets the current position of the motor
		void set_angle(int16_t angle_in);	// Sets the current angle of the motor

		void zero_esum_l(void);		// Resets esum_l to zero
		void zero_esum_a(void);		// Resets esum_a to zero 
		diagnostic run(bool en_p, bool en_i, bool en_d, bool op_type);	// Calculates pwm signal. Needs to be told what control
};

#endif /* MOTOR_DRIVER_H_ */