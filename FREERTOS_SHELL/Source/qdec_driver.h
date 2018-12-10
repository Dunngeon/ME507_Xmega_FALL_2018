/* This file has been prepared for Doxygen automatic documentation generation. */
/*! \file *********************************************************************
 *
 * \brief  The XMEGA Quadrature Decoder driver header file.
 *
 *      This file contains the function prototypes and enumerator definitions
 *      for various configuration parameters for the XMEGA Quadrature Decoder.
 *
 *      The driver is not intended for size and/or speed critical code. The
 *      driver is intended for rapid prototyping and documentation purposes for
 *      getting started with the XMEGA Quadrature Decoder.
 *
 *      For size and/or speed critical code, it is recommended to copy the
 *      function contents directly into your application instead of making
 *      a function call.
 *
 * \par Application note:
 *      AVR1600: Using the XMEGA Quadrature Decoder
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation, now Microchip Technology Inc.: http://www.microchip.com \n
 *
 * $Revision: 7623 $
 * $Date: 2017-09-06 14:57:02 +0200 (on, 06 sep 2017) $
 *
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
/*
#ifdef __cplusplus
extern "C" {
	#endif
*/
#ifndef __QDEC_DRIVER_H__
#define __QDEC_DRIVER_H__

#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Port I/O for SFR's
#include <stdbool.h>

#define CW_DIR   0 /* Clockwise direction. */
#define CCW_DIR  1 /* Counter Clockwise direction. */
//clock divison on the counter
//#define CLOCK_DIV_bm  TC_CLKSEL_DIV64_gc
//#define CLOCK_DIV     64

/* Definitions of macros. */

/*! \brief This macro return the value of the capture register.
 *
 * \param  _tc   The Timer/Counter to get the capture value from.
 */
#define GetCaptureValue(_tc)  ( _tc.CCA )


/* Prototyping of functions. */

bool QDEC_Total_Setup(PORT_t * qPort,
                      uint8_t qPin,
                      bool invIO,
                      uint8_t qEvMux,
                      EVSYS_CHMUX_t qPinInput,
                      bool useIndex,
                      EVSYS_QDIRM_t qIndexState,
                      TC0_t * qTimer,
                      TC_EVSEL_t qEventChannel,
                      uint16_t lineCount);

//TC1 type total setup					  
bool QDEC_Total_Setup(PORT_t * qPort,
uint8_t qPin,
bool invIO,
uint8_t qEvMux,
EVSYS_CHMUX_t qPinInput,
bool useIndex,
EVSYS_QDIRM_t qIndexState,
TC1_t * qTimer,
TC_EVSEL_t qEventChannel,
uint16_t lineCount);

bool QDEC_Port_Setup(PORT_t * qPort, uint8_t qPin, bool useIndex, bool invIO);

bool QDEC_EVSYS_Setup(uint8_t qEvMux,
                      EVSYS_CHMUX_t qPinInput,
                      bool useIndex,
                      EVSYS_QDIRM_t qIndexState );

void QDEC_TC_Dec_Setup(TC0_t * qTimer,
                       TC_EVSEL_t qEventChannel,
                       uint16_t countersize);
void QDEC_TC_Dec_Setup(TC1_t * qTimer, TC_EVSEL_t qEventChannel, uint16_t countersize);
void QDEC_TC_Freq_Setup(TC0_t * qTimer,
                        TC_EVSEL_t qEventChannel,
                        EVSYS_CHMUX_t qPinInput,
                        TC_CLKSEL_t clksel);

uint8_t QDEC_Get_Direction(TC0_t * qTimer);
uint16_t QDEC_Read_TC(TC0_t * qTimer);
uint16_t QDEC_Read_TC(TC1_t *qTimer);

#endif /* __QDEC_DRIVER_H__ */

/*
#ifdef __cplusplus
}
#endif
*/