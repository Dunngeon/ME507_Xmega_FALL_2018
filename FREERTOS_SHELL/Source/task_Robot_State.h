//**************************************************************************************
/** \file task_Robot_State.h
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

// This define prevents this .h file from being included multiple times in a .cpp file
#ifndef _TASK_ROBOT_STATE_H_
#define _TASK_ROBOT_STATE_H_

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

#include "qdec_driver.h"					//quadrature encoder driver
#include <math.h>							//Math library for trignometric functions

#define TICKSPERINCH 77			//this defined value relates ticks of encoder to linear distance on the 2D plane, accounting for wheel diameter. UNITS: ticks/inch
#define DELAYINTERVAL_MS 5    //This defines the interval that the task will run on.
#define WHEELBASE_INCH 10		//This defines the wheelbase of the robot in inches
#define WHEELBASE_TICKS	531		//This defines the wheelbase of the robot in ticks

class task_Robot_State : public frt_task
{
private:
	// No private variables or methods for this class

protected:
	/*enum Robot_states {
		INIT,
		LED_,
		ROBOT_S2,
		ROBOT_S3
	};	*/				//!< Task state
	uint8_t ctr;		//!< Loop counter
	//Encoder value storage variables -> Current and previous state
	int16_t M_Enc1_Val;
	int16_t M_Enc2_Val;
	int16_t M_Enc1_Val_Prev;
	int16_t M_Enc2_Val_Prev;
	
	int16_t M_1_DistTick;
	int16_t M_2_DistTick;
	int16_t M_DifferTick;
	
	int16_t M_1_v1; 
	int16_t M_2_v2; 
	int16_t R_V_Bar; 
	int16_t R_Omega;
	int16_t R_POS_Y_delta; //robot only moves in Y direction in the local coordinate system
	int16_t R_THETA_Delta;		//
	int16_t R_THETA_PREV; //previous angular positon of the robot.
	
	int16_t R_INERT_Theta;	
	int16_t R_I_POS_X_delta; 
	int16_t R_I_POS_Y_delta;
	int16_t R_I_POS_X;
	int16_t R_I_POS_Y;
public:
	// This constructor creates a user interface task object
	task_Robot_State (const char*, unsigned portBASE_TYPE, size_t, emstream*);

	/** This method is called by the RTOS once to run the task loop for ever and ever.
	 */
	void run (void);
};

#endif // _TASK_ROBOT_STATE_H_
