/*
 * FollowInstructions.h
 *
 * Created: 12/6/2018 12:29:25 AM
 *  Author: Alec
 */ 


#ifndef FOLLOWINSTRUCTIONS_H_
#define FOLLOWINSTRUCTIONS_H_

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

class FollowInstructions : public frt_task
{
	public:
		FollowInstructions (const char*, unsigned portBASE_TYPE, size_t, emstream*);
		void run();
		
		char curChar, prevChar;
		int curSpeed;
		
};



#endif /* FOLLOWINSTRUCTIONS_H_ */