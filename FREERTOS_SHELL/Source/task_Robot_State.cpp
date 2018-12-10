//**************************************************************************************
/** \file task_Robot_State.cpp
 *    This file contains header stuff for a 2wd robot with caster wheel encoder read/state estimator.
 *
 *  Revisions:
 *    \li 12-05-2018 RGD - Created task Robot State.
 *
 *  License:
 *    This file is copyright 2018 by RG Dunn and released under the Lesser GNU 
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
#include "task_Robot_State.h"                      // Header for this file

// Initializing encoder-based positions
int16_t Robot_Pos_X_INERT = 0;			// Contains current position of robot in X_INERTIAL
int16_t Robot_Pos_Y_INERT = 0;			// Contains current position of robot in Y_INERTIAL
int16_t Robot_Angle_Theta_INERT = 0;	// Contains current angle of the robot in THETA_INERTIAL
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

task_Robot_State::task_Robot_State (const char* a_name, 
					  unsigned portBASE_TYPE a_priority, 
					  size_t a_stack_size,
					  emstream* p_ser_dev
					 )
	: frt_task (a_name, a_priority, a_stack_size, p_ser_dev)
{
	//This constructor does nothing.
	
}


//-------------------------------------------------------------------------------------


void task_Robot_State::run (void)
{
	// Make a variable which will hold times to use for precise task scheduling
	portTickType previousTicks = xTaskGetTickCount ();

	// Wait a little while for user interface task to finish up
	delay_ms(10);
	bool success = false; //declare bool for checking success of function calls
	//configure both quadrature counter elements
	
	while(1)
	{
		switch (state)
		{
		case (0):
			//setting up quadrature encoders!
			// ENC1 is Left side of bot
			// ENC2 is right side of bot
			
			success = QDEC_Total_Setup(&PORTD, 4, false, 0, EVSYS_CHMUX_PORTD_PIN4_gc, false, EVSYS_QDIRM_00_gc, &TCD1, TC_EVSEL_CH0_gc, 0xFFFF); //setup M_ENC1 quad. encoder
			//*p_serial << "ENC1 Setup Success? " << success << endl;
			success = false;
			success = QDEC_Total_Setup(&PORTE, 4, false, 2, EVSYS_CHMUX_PORTE_PIN4_gc, false, EVSYS_QDIRM_00_gc, &TCF0, TC_EVSEL_CH2_gc, 0xFFFF); //setup M_ENC2 quad. encoder
			//*p_serial << "ENC2 Setup Success? " << success << endl;
			M_Enc1_Val_Prev = QDEC_Read_TC(&TCD1); //read value of encoders for starting value
			M_Enc2_Val_Prev = QDEC_Read_TC(&TCF0);
			R_THETA_Delta = 0; //zero out the angular position of the robot upon startup.
			R_I_POS_X = 0;
			R_I_POS_Y = 0;
			transition_to(1);
			break;
			
		case (1):
				//get current encoder values
				M_Enc1_Val = -1 * QDEC_Read_TC(&TCD1); //multiply by -1 so that positive encoder count is forwards on both sides.
				M_Enc2_Val = QDEC_Read_TC(&TCF0);
				
				//calculate ticks elapsed since last iteration, we'll left shift by 2 bits to increase data resolution (at max speed, we run the risk of losing MSB data, should quantify this risk)
				M_1_DistTick = ((M_Enc1_Val-M_Enc1_Val_Prev) << 2);
				M_2_DistTick = ((M_Enc2_Val-M_Enc2_Val_Prev) << 2);
				//calculate differential ticks between the motors.
				//Positive M_DifferTick = CCW Rotation (right faster than left)
				//Negative M_DifferTick = CW Rotation (left side faster than right)
				M_DifferTick = (M_2_DistTick - M_1_DistTick); //left shifted by two from earlier
				*p_serial << "Differential Encoder Values" << M_1_DistTick << " " << M_2_DistTick << endl;
				
				//remain in tick units (leftshifted two) to maintain maximal resolution
				//calculate v1 & v2 (ticks/timetasktakestorun)
				M_1_v1 = M_1_DistTick / (DELAYINTERVAL_MS << 2);			//calculate linear velocity of left wheel (ticks/ms)
				M_2_v2 = M_2_DistTick / (DELAYINTERVAL_MS << 2);			//calculate linear velocity of right wheel (ticks/ms)
				*p_serial << "M vel 1" << hex << (M_1_v1>>0) << " " << (M_1_v1>>1)  << " M2 V " << (M_2_v2) << " " << (M_2_v2 >> 1) << endl;
				//calculate vbar and angular position of the drivebase in robot coordinates.
				R_V_Bar = (M_2_v2 + M_1_v1) / (2<<2);					//calculate Vbar for the robot (ticks/ms)
				R_POS_Y_delta = ((M_2_v2 + M_1_v1) / (2<<2)) * (DELAYINTERVAL_MS << 2); //delta y position in local frame
				R_Omega = (M_2_v2 - M_1_v1) / (WHEELBASE_TICKS << 2);	//angular velocity (rad/ms) -> only calculated for debugging. Angular position is calculated directly.
				R_THETA_Delta = ((M_2_v2 - M_1_v1) * (DELAYINTERVAL_MS << 2)) / (WHEELBASE_TICKS<<2);		//calculate angular position change for the robot (we calculate directly to avoid losing resolution (rad))
				
				*p_serial << "R_V_Bar " << hex << R_V_Bar << " RPOSYDELTA " << R_POS_Y_delta << " ROMEGA " << R_Omega << " R_THETA_DELTA " << R_THETA_Delta << endl;
				R_INERT_Theta = R_INERT_Theta + (R_THETA_Delta>>2); //update the angular position of the robot with the new estimate (right shift two for compatibility).
				//now translate R_POS_Y_delta back to inertial frame
				
				R_I_POS_X_delta = R_POS_Y_delta * cos(R_INERT_Theta);	//calculate vbar component in x inertial frame
				R_I_POS_Y_delta = R_POS_Y_delta * sin(R_INERT_Theta);	//calculate vbar component in y intertial frame
				R_I_POS_X = R_I_POS_X + (R_I_POS_X_delta>>2);				//compute new robot position in X inertial
				R_I_POS_Y = R_I_POS_Y + (R_I_POS_Y_delta>>2);				//compute new robot position in Y inertial
				
				//we are back to regular no leftshift data!
				//output X,Y,Theta to motordriver task via shares.
				Robot_Pos_X_INERT = R_I_POS_X;
				Robot_Pos_Y_INERT = R_I_POS_Y;
				Robot_Angle_Theta_INERT = R_INERT_Theta;
				
				
				
				M_Enc1_Val_Prev = M_Enc1_Val;
				M_Enc2_Val_Prev = M_Enc2_Val;
				runs++;
				delay_from_to_ms(previousTicks,DELAYINTERVAL_MS);
			
			
		}
		runs++;
		delay_from_to_ms(previousTicks,10);
	}
}