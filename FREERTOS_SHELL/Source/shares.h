//*************************************************************************************
/** \file shares.h
 *    This file contains extern declarations for queues and other inter-task data
 *    communication objects used in a ME405/507/FreeRTOS project. 
 *
 *  Revisions:
 *    \li 09-30-2012 JRR Original file was a one-file demonstration with two tasks
 *    \li 10-05-2012 JRR Split into multiple files, one for each task plus a main one
 *    \li 10-29-2012 JRR Reorganized with global queue and shared data references
 *
 *  License:
 *		This file is copyright 2012 by JR Ridgely and released under the Lesser GNU 
 *		Public License, version 2. It intended for educational use only, but its use
 *		is not limited thereto. */
/*		THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *		AND	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * 		IMPLIED 	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * 		ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * 		LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUEN-
 * 		TIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 * 		OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * 		CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * 		OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * 		OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//*************************************************************************************

// This define prevents this .h file from being included multiple times in a .cpp file
#ifndef _SHARES_H_
#define _SHARES_H_


//-------------------------------------------------------------------------------------
// Externs:  In this section, we declare variables and functions that are used in all
// (or at least two) of the files in the data acquisition project. Each of these items
// will also be declared exactly once, without the keyword 'extern', in one .cpp file
// as well as being declared extern here. 

/**
 * \var print_ser_queue
 * \brief This queue allows tasks to send characters to the user interface task for display.
 */
extern frt_text_queue print_ser_queue;			// This queue allows tasks to send characters to the user interface task for display.


// Motor-related shares
//-----------------------------------------------
//-----------------------------------------------
/**
 * \var Robot_Pos_X_INERT
 * \brief This share contains the current X position of the robot.
 */
 extern int16_t Robot_Pos_X_INERT;			// Contains current position of robot in X_INERTIAL
 

 
 /**
 * \var Robot_Pos_Y_INERT
 * \brief This share contains the current Y position of the robot.
 */
 extern  int16_t Robot_Pos_Y_INERT;			// Contains current position of robot in Y_INERTIAL
 //---------

 /**
 * \var Robot_Angle_Theta_INERT
 * \brief This share contains the current angle of the robot.
 */
 extern int16_t Robot_Angle_Theta_INERT;	// Contains current angle of the robot in THETA_INERTIAL
 /**
  
 /**
 * \var setpoint_1
 * \brief This share contains the current setpoint of motor 1.
 */
 extern int16_t setpoint_l_1;		// Contains current setpoint of motor 1
 /**
 * \var setpoint_2
 * \brief This share contains the current setpoint of motor 2.
 */
 extern int16_t setpoint_l_2;		// Contains current setpoint of motor 2

 //------------------------

 /*
 * \var angle_2
 * \brief This share contains the current angle of motor 2.
 */
 extern int16_t angle_2;			// Contains current angle of motor 2
 
  //---------

 /**
 * \var setpoint_a_1
 * \brief This share contains the current angular setpoint of motor 1.
 */
 extern int16_t setpoint_a_1;		// Contains current angular setpoint of motor 1
  /**
 * \var setpoint_a_2
 * \brief This share contains the current angular setpoint of motor 2.
 */
 extern int16_t setpoint_a_2;		// Contains current angular setpoint of motor 2

 //-----------------------------------------------
 //-----------------------------------------------

 // Serial diagnostic shares
  /**
 * \var pwm_tot_1
 * \var pwm_tot_2
 * \var esum_l_1
 * \var esum_l_2
 * \var esum_a_1
 * \var esum_a_2
 * \var pwm_lin_1
 * \var pwm_lin_2
 * \var pwm_ang_1
 * \var pwm_ang_2
 * \brief These shares are for diagnostic purposes only.
 */
 extern int16_t pwm_tot_1;
 extern int16_t pwm_tot_2;
 extern int16_t esum_l_1;
 extern int16_t esum_l_2;
 extern int16_t esum_a_1;
 extern int16_t esum_a_2;
 extern int16_t pwm_lin_1;
 extern int16_t pwm_lin_2;
 extern int16_t pwm_ang_1;
 extern int16_t pwm_ang_2;
 extern int16_t LinearDistance;



#endif // _SHARES_H_
