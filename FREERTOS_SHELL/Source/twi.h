#ifdef __cplusplus
extern "C" {
	#endif

//**************************************************************************************
/** \file twi.h
 *    This file contains header info for I2C functions on an ATxmega
 *    
 *
 *  Revisions:
 *    \li Created by unknown.
 *	  \li 11-22-2018 Ryan G. Dunn - Began testing with ATxmega128A3U
 *	  \li 11-30-2018 RGD - Added additional error register flags for writing, modified timeout on read to support BNO080s obtuse clock stretching.
 *    \li 12-01-2018 RGD - Verified working with xmega 128A3U, added TWI_read_clear command
 *    \li 12-02-2018 RGD - Added double buffer read function
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
//#include <stdio.h>							// include standard I/O

#ifndef TWI_H_
#define TWI_H_

#include <avr/io.h>                         // Port I/O for SFR's
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>

//define constants
#define TWI						TWIE
#define TWI_MASTER_vect			TWIE_TWIM_vect
#define TWI_SLAVE_vect			TWIE_TWIS_vect

#define TWI_BAUD_REG			155		// 100KHz, Baud = (Fclk/(2*Ftwi)) -5
#define TWI_MASTER_INTLVL_gc	TWI_MASTER_INTLVL_MED_gc
#define TWI_IDLE_TIMEOUT_MS		10

//#define TWI_INTERRUPT_DRIVEN



extern void TWI_init(void);
//extern bool TWI_write_reg(uint8_t address, uint8_t reg, const uint8_t *buffer, uint8_t buffer_size);
extern bool TWI_write_reg(uint8_t address, uint8_t reg, const uint8_t *header, uint8_t header_size, const uint8_t *buffer, uint8_t buffer_size);
extern bool TWI_read_reg(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t buffer_size);
extern bool TWI_read(uint8_t address, uint8_t *buffer, uint8_t buffer_size);
extern bool TWI_read_double_buff(uint8_t address, uint8_t *header_buff, uint8_t header_size, uint8_t *buffer, uint8_t buffer_size);
extern bool TWI_read_clear(uint8_t address, uint16_t buffer_size); //reads massive amount of data from device without storing it. Provide a large buffer size and when the device has no data to send it will timeout.
//extern int TWI_scan(void); //scans for devices on the I2C bus, if found, returns their location. Otherwise, returns 0.



#endif /* TWI_H_ */

#ifdef __cplusplus
}
#endif