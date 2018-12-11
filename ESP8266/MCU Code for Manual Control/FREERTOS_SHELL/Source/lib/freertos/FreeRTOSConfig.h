/*
    FreeRTOS V7.1.1 - Copyright (C) 2012 Real Time Engineers Ltd.
	

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!
    
    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?                                      *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    
    http://www.FreeRTOS.org - Documentation, training, latest information, 
    license and contact details.
    
    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool.

    Real Time Engineers ltd license FreeRTOS to High Integrity Systems, who sell 
    the code with commercial support, indemnification, and middleware, under 
    the OpenRTOS brand: http://www.OpenRTOS.com.  High Integrity Systems also
    provide a safety engineered and independently SIL3 certified version under 
    the SafeRTOS brand: http://www.SafeRTOS.com.
*/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <avr/io.h>

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE. 
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

/** This define sets the rate at which RTOS tick interrupts will occur. The tick
 *  interrupt controls the resolution of task switching, so if you have very short,
 *  high priority tasks this rate should be pretty high. However, a high tick rate
 *  causes the RTOS to take up more CPU time, so don't overdo it. 
 */
#define configTICK_RATE_HZ              ( ( portTickType ) 1000 )

/** This define enables the preemptive RTOS scheduler. Without it, there is no
 *  preemptive scheduling, so only cooperative scheduling can be used. 
 */
#define configUSE_PREEMPTION            1

/** This define informs FreeRTOS of the CPU clock crystal frequency. For ME405 style
 *  code, we just copy the value of F_CPU (which is set in the Makefile) here. 
 */
#define configCPU_CLOCK_HZ              ( F_CPU )

/** This macro allows one to put in a number of milliseconds for a time interval and
 *  get out the correct number of RTOS ticks to match (approximately) that interval.
 *  For example, if one has a delay function, one can call
 *  \code delay (configMS_TO_TICKS (15)); \endcode
 *  to get a 15 ms delay regardless of the configured tick rate. If the requested
 *  delay comes out to less than one tick, this macro causes a delay of one tick.
 */
#define configMS_TO_TICKS(x)            ((((x) * configTICK_RATE_HZ / 1000) > 0) \
                                        ? ((x) * configTICK_RATE_HZ / 1000) : 1)

/** This define is set to compile some extra code that helps keep track of memory and
 *  processor usage in tasks. It does not check for state transitions in tasks. Since
 *  tracing takes up memory and processor time, it should only be used for debugging.
 */
#define configUSE_TRACE_FACILITY        0

/** This define causes task run times to be measured by the RTOS profiler. This is a
 *  useful debugging feature, but it takes up memory and processor time, so it should
 *  only be used when debugging the performance of a program.
 */
#define configGENERATE_RUN_TIME_STATS   0

/** This define sets the maximum number of task priorities available for use. More
 *  memory is used if a higher number of priorities is set, so you should not make
 *  more priorities available than are needed. Since many tasks can share the same
 *  priority, this number generally does not need to be more than 3 to 5 or so. 
 */
#define configMAX_PRIORITIES            ( ( unsigned portBASE_TYPE ) 4 )

/** This define sets the size of the stack used by the idle task. It is also common
 *  for a user to set other task's stack sizes to this same value when calling
 *  xTaskCreate(). The smallest value known to be used for AVR's is 85, but a larger
 *  value is commonly used with processors that have more memory. 
 */
#define configMINIMAL_STACK_SIZE        ( ( unsigned short ) 100 )

/** This define sets the size of the block of memory from which all dynamically 
 *  allocated memory is allocated -- including task stacks, queues, and whatever the
 *  user's functions need (but not the main system stack). Since the amount of SRAM
 *  available depends on the processor, we'll use a formula to estimate the right
 *  amount of SRAM to use for the RTOS heap. For a 2K chip such as the ATmega32, we
 *  end up using about half of SRAM; for a larger chip, we use about 3/4 of the total 
 *  data memory for the heap. The default from FreeRTOS for the ATmega323 is 2500 
 *  bytes, which seems strange because the data sheets say is only has 2K of SRAM. 
 *  This formula is intended to be altered by the user for different configurations.
 */
#define configTOTAL_HEAP_SIZE           ( ( ( (uint32_t) RAMEND - (uint32_t) RAMSTART ) * 2 ) / 4 ) 

/** This define sets the maximum length of task names, plus one byte for the '\0'
 *  which signifies the end of the string. When set to 8, it allows 7-letter names.
 */
#define configMAX_TASK_NAME_LEN         ( 10 )

/** This define enables use of vApplicationIdleHook() to run a task (or a set of
 *  "co-routines", cooperatively scheduled tasks) at the lowest priority.
 */
#define configUSE_IDLE_HOOK             0

/** This define enables the use of vApplicationTickHook(), which runs within the
 *  RTOS tick timer interrupt. Code which does timing tasks can be put here. This
 *  functionality is seldom used.
 */
#define configUSE_TICK_HOOK             0

/** When this define is set to 1, the RTOS tick counter will only be 16 bits in size.
 *  This makes the RTOS tick interrupt a little quicker and saves some memory, but
 *  the tick counter overflows very quickly and isn't useful for measuring real time.
 *  For ME405/507 use, we generally set this to 0 to use a 32 bit tick counter. 
 */
#define configUSE_16_BIT_TICKS          0

/** This define causes the idle task to yield whenever there is a preemptively 
 *  scheduled RTOS task at idle priority ready to run. It only affects the behavior of
 *  the idle task with respect to RTOS tasks which are at idle priority; RTOS tasks at
 *  a higher priority always get to take over from the idle task.
 */
#define configIDLE_SHOULD_YIELD         1

/** This define is only used with an RTOS kernel aware debugger, which we don't have 
 *  in ME405/507. When used, it allows information such as queue names to be kept for
 *  sharing with the debugger.
 */
#define configQUEUE_REGISTRY_SIZE       0

/** This define must be set to 1 to allow mutexes to be used in your program. Since 
 *  mutexes are necessary in most preemptively scheduled programs, we almost always
 *  set this to 1 to enable mutexes.
 */
#define configUSE_MUTEXES               1

/** The RAM pointer size on an AVR processor is 16 bits; set it here to shut up a dumb
 *  compiler warning that comes out in tasks.c if the default 32 bits is used. 
 */
#define portPOINTER_SIZE_TYPE           uint16_t

/** This define is set to 1 in order to allow the use of co-routines, which are a sort
 *  of cooperatively multitasked set of tasks.
 */
#define configUSE_CO_ROUTINES           0

/** This is the maximum number of co-routines to use. Each takes memory, so this 
 *  define should not be set arbitrarily high. 
 */
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/* Set each of the following definitions to 1 to include the corresponding API 
 * function, or to zero to exclude the API function. 
 */

#define INCLUDE_vTaskPrioritySet                 1
#define INCLUDE_uxTaskPriorityGet                1
#define INCLUDE_vTaskDelete                      0
#define INCLUDE_vTaskCleanUpResources            0
#define INCLUDE_vTaskSuspend                     0
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_pcTaskGetTaskName                1
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_xTaskGetIdleTaskHandle           1

#endif /* FREERTOS_CONFIG_H */
