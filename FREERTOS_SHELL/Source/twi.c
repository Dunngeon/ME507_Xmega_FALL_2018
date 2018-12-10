//**************************************************************************************
/** \file twi.cpp
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
/*	  TODO: replace dependency upon util/delay.h timers with FRTOS timers.
 *		allow user to specify which TWI Port they want						*/
//**************************************************************************************

 

#include "twi.h"


enum TRANSACTION_TYPES_enum
{
	TWI_TRANSACTION_IDLE,
	TWI_TRANSACTION_WRITE_REG,
	TWI_TRANSACTION_READ_REG,
	TWI_TRANSACTION_READ
};

enum TRANSACTION_RESULTS_enum
{
	TWI_RESULT_PENDING,
	TWI_RESULT_FAILED,
	TWI_RESULT_OK
};

enum STATE_MACHINE_enum
{
	TWI_STATE_REGISTER,
	TWI_STATE_RESTART,
	TWI_STATE_DATA
};

#define TWI_READ_bm				(1<<0)



volatile uint8_t	transaction_type_AT = TWI_TRANSACTION_IDLE;
volatile uint8_t transaction_result_AT = TWI_RESULT_PENDING;
volatile uint8_t state_AT = TWI_STATE_REGISTER;
volatile uint8_t	device_address_AT = 0;
volatile uint8_t	reg_address_AT = 0;
volatile uint8_t data_bytes_AT = 0;
volatile uint8_t	*data_buffer_AT = 0;


//If not using interrupt driven i2c, this code will run.
#ifndef TWI_INTERRUPT_DRIVEN


//-------------------------------------------------------------------------------------
/** This function initializes TWI communication on PORTE in Master mode
 *  
 */
void TWI_init(void)
{
	TWI.CTRL = 0;	// SDA hold time off
	
	//added code from complicated i2c lib
	PORTE.DIRSET = PIN0_bm | PIN1_bm;
	PORTE.PIN0CTRL = PORT_OPC_WIREDANDPULL_gc; //SDA pull up output
	PORTE.PIN1CTRL = PORT_OPC_WIREDANDPULL_gc; //SCL pull up output
	TWIE.MASTER.BAUD = 115;
	TWIE.MASTER.STATUS |= TWI_MASTER_RIF_bm | TWI_MASTER_WIF_bm | TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm | TWI_MASTER_BUSSTATE_IDLE_gc; //clear all flags initially and select bus state IDLE
	//end added code
	TWIE.MASTER.CTRLB = TWI_MASTER_TIMEOUT_200US_gc;
	TWIE.MASTER.CTRLC = 0;
	//TWI.MASTER.BAUD = TWI_BAUD_REG;
	TWI.MASTER.CTRLA = TWI_MASTER_ENABLE_bm;
	//TWI.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
	
}


//-------------------------------------------------------------------------------------
/** This function writes a set of data to an i2c device and optional register.
 *  @param address Address of the i2c slave device (0-127)
 *  @param reg	Optional register address on the i2c device
 *  @param *header Pointer to the data_array containing the header data bytes
 *  @param header_size Size of the header array to write
 *  @param *buffer Point to the data array containing the buffer bytes
 *  @param buffer_size Size of the buffer array to write
 *  
 */
bool TWI_write_reg(uint8_t address, uint8_t reg, const uint8_t *header, uint8_t header_size, const uint8_t *buffer, uint8_t buffer_size)
{
	__label__ failure;
	bool success = false;
	uint8_t	i;

	// wait for bus to become idle
	i = 0;
	while ((TWI.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) != TWI_MASTER_BUSSTATE_IDLE_gc)
	{
		if (i > TWI_IDLE_TIMEOUT_MS)
			goto failure;
		_delay_ms(1);
		i++;
	}

	// start write
	TWI.MASTER.ADDR = address << 1;		// sends start and address
	while ((TWI.MASTER.STATUS & TWI_MASTER_WIF_bm) == 0);
	if (TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm | TWI_MASTER_RXACK_bm))
		goto failure;

	//if device needs a register address, write it now, otherwise skip. 
	//if(reg != 0)
	//{
		//TWI.MASTER.DATA = reg;	
	//}
	
	// header bytes
	for (i = 0; i < header_size; i++)
	{
		while ((TWI.MASTER.STATUS & TWI_MASTER_WIF_bm) == 0);

		if (TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm | TWI_MASTER_RXACK_bm))
		goto failure;

		TWI.MASTER.DATA = header[i];
	}
	while ((TWI.MASTER.STATUS & TWI_MASTER_WIF_bm) == 0); //redundant feature


	// data bytes
	for (i = 0; i < buffer_size; i++)
	{
		while ((TWI.MASTER.STATUS & TWI_MASTER_WIF_bm) == 0);

		if (TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm | TWI_MASTER_RXACK_bm))
			goto failure;

		TWI.MASTER.DATA = buffer[i];
	}
	while ((TWI.MASTER.STATUS & TWI_MASTER_WIF_bm) == 0);

	// transaction complete or failed
	success = true;
failure:
	TWI.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;

	return success;
}

//-------------------------------------------------------------------------------------
/** This function reads a set of data from an i2c device and optional register.
 *  @param address Address of the i2c slave device (0-127)
 *  @param reg	Optional register address on the i2c device
 *  @param *buffer Pointer to the data array to write to
 *  @param buffer_size number of bytes to read to the buffer
 *  
 */
bool TWI_read_reg(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t buffer_size)
{
	__label__ failure;
	uint8_t	timeout;

	// wait for bus to become idle
	timeout = 0;
	while ((TWI.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) != TWI_MASTER_BUSSTATE_IDLE_gc)
	{
		if (timeout > TWI_IDLE_TIMEOUT_MS)
			goto failure;
		_delay_ms(1);
		timeout++;
	}

	// start write
	TWI.MASTER.ADDR = address << 1;		// sends start and address
	while ((TWI.MASTER.STATUS & TWI_MASTER_WIF_bm) == 0);
	if (TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm | TWI_MASTER_RXACK_bm))
		goto failure;

	// register address
	if(reg != 0)
	{
		
	
		TWI.MASTER.DATA = reg;
		while ((TWI.MASTER.STATUS & TWI_MASTER_WIF_bm) == 0);
		if (TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm))
			goto failure;
	}
	// (re)start read
	TWI.MASTER.ADDR = (address << 1) | TWI_READ_bm;		// sends repeated start and address

	// read requested number of bytes
	timeout = 200;	// double timeout to account for read setup byte
	while (buffer_size)
	{
		while ((TWI.MASTER.STATUS & (TWI_MASTER_RIF_bm | TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm)) == 0)
		{
			timeout--;
			if (timeout == 0)
				goto failure;
			_delay_us(1);
		}
		timeout = 100;

		*buffer++ = TWI.MASTER.DATA;

		if (TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm))
			goto failure;
		buffer_size--;
		if (buffer_size != 0)
			TWI.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;							// send ACK
		else
			TWI.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;		// send NACK
	}

	// transaction complete or failed
	return true;
failure:
	TWI.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
	return false;
}


//-------------------------------------------------------------------------------------
/** This function reads a set of data from an i2c device.
 *  @param address Address of the i2c slave device (0-127)
 *  @param *buffer Pointer to the data array to write to
 *  @param buffer_size number of bytes to read to the data array
 *  
 */
bool TWI_read(uint8_t address, uint8_t *buffer, uint8_t buffer_size)
{
	__label__ failure;
	uint16_t	timeout;

	// wait for bus to become idle
	timeout = 0;
	while ((TWI.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) != TWI_MASTER_BUSSTATE_IDLE_gc)
	{
		if (timeout > 10)
			goto failure;
		_delay_ms(1);
		timeout++;
	}

	// start read
	TWI.MASTER.ADDR = (address << 1) | TWI_READ_bm;		// sends start and address

	// read requested number of bytes
	timeout = 0xFFF;	// double timeout to account for read setup byte
	while (buffer_size)
	{
		while ((TWI.MASTER.STATUS & (TWI_MASTER_RIF_bm | TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm)) == 0)
		{
			timeout--;
			if (timeout == 0)
				goto failure;
			_delay_us(1);
		}
		timeout = 0xFFF;

		*buffer++ = TWI.MASTER.DATA;

		if (TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm))
			goto failure;
		buffer_size--;
		if (buffer_size != 0)
			TWI.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;							// send ACK
		else
			TWI.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;		// send NACK
		
	}
	// transaction complete or failed
	return true;
	failure:
	TWI.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;

	return false;
}



//-------------------------------------------------------------------------------------
/** This function reads a set of data from an i2c device and writes to two independent buffers.
 *  @param address Address of the i2c slave device (0-127)
 *  @param *header Pointer to the first data array to write
 *  @param header_size Number of bytes to read to the first array
 *  @param *buffer Pointer to the second data array to write to
 *  @param buffer_size number of bytes to read to the second array
 *  
 */
bool TWI_read_double_buff(uint8_t address, uint8_t *header_buff, uint8_t header_size, uint8_t *buffer, uint8_t buffer_size)
{
	__label__ failure;
	uint16_t	timeout;

	// wait for bus to become idle
	timeout = 0;
	while ((TWI.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) != TWI_MASTER_BUSSTATE_IDLE_gc)
	{
		if (timeout > 10)
			goto failure;
		_delay_ms(1);
		timeout++;
	}

	// start read
	TWI.MASTER.ADDR = (address << 1) | TWI_READ_bm;		// sends start and address

	// read requested number of bytes into buffer 1
	timeout = 0xFFF;	// double timeout to account for read setup byte
	while (header_size)
	{
		while ((TWI.MASTER.STATUS & (TWI_MASTER_RIF_bm | TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm)) == 0)
		{
			timeout--;
			if (timeout == 0)
			goto failure;
			_delay_us(1);
		}
		timeout = 0xFFF;

		*header_buff++ = TWI.MASTER.DATA;

		if (TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm))
			goto failure;
		header_size--;
		if (header_size != 0)
			TWI.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;							// send ACK
		
		//do nothing since we still want to read the remaining bytes into the other buffer
		//TWI.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;		// send NACK
		
	}
	timeout = 0xFFF;
	//read any remaining bytes into buffer 2
	while (buffer_size)
	{
		while ((TWI.MASTER.STATUS & (TWI_MASTER_RIF_bm | TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm)) == 0)
		{
			timeout--;
			if (timeout == 0)
				goto failure;
			_delay_us(1);
		}
		timeout = 0xFFF;

		*buffer++ = TWI.MASTER.DATA;

		if (TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm))
			goto failure;
		buffer_size--;
		if (buffer_size != 0)
			TWI.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;							// send ACK
		else
			TWI.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;		// send NACK
		
	}
	// transaction complete or failed
	return true;
	failure:
	TWI.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;

	return false;
}


//-------------------------------------------------------------------------------------
/** This function reads data from an i2c device blasts it into the bit trash bucket.
 *  @param address Address of the i2c slave device (0-127)
 *  @param buffer_size number of bytes to read from the i2c device (bigger is better, it'll time out if no more bytes are being sent)
 *  
 */
bool TWI_read_clear(uint8_t address, uint16_t buffer_size)
{
	__label__ failure;
	uint16_t	timeout;
	uint8_t		throwawaydata;
	// wait for bus to become idle
	timeout = 0;
	while ((TWI.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) != TWI_MASTER_BUSSTATE_IDLE_gc)
	{
		if (timeout > 10)
			goto failure;
		_delay_ms(1);
		timeout++;
	}

	// start read
	TWI.MASTER.ADDR = (address << 1) | TWI_READ_bm;		// sends start and address

	// read requested number of bytes
	timeout = 0xFFF;	// double timeout to account for read setup byte
	while (buffer_size)
	{
		while ((TWI.MASTER.STATUS & (TWI_MASTER_RIF_bm | TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm)) == 0)
		{
			timeout--;
			if (timeout == 0)
				goto failure;
			_delay_us(1);
		}
		timeout = 0xFFF;

		
		throwawaydata = TWI.MASTER.DATA;
		if (TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm))
			goto failure;
		buffer_size--;
		if (buffer_size != 0)
			TWI.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;							// send ACK
		else
			TWI.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;		// send NACK
	}




	// transaction complete or failed
	return true;
failure:
	TWI.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;

	return false;
}


#endif


//this setup only occurs if TWI_INTERRUPT_DRIVEN is defined.
//UNTESTED WITH XMEGA
#ifdef TWI_INTERRUPT_DRIVEN


/**************************************************************************************************
* Set up the TWI peripheral in Master mode
*/
void TWI_init(void)
{
	PR.PRPC	&= ~PR_TWI_bm;
	TWI.CTRL = 0;	// SDA hold time off
	
	TWI.MASTER.CTRLB = TWI_MASTER_TIMEOUT_200US_gc;
	TWI.MASTER.CTRLC = 0;
	TWI.MASTER.BAUD = TWI_BAUD_REG;
	TWI.MASTER.CTRLA = TWI_MASTER_INTLVL_gc | TWI_MASTER_RIEN_bm | TWI_MASTER_WIEN_bm | TWI_MASTER_ENABLE_bm;
	TWI.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
}

/**************************************************************************************************
* Master mode interrupt handler
*/
ISR(TWI_MASTER_vect)
{
	__label__ reset;
	
	switch(transaction_type_AT)
	{
		case TWI_TRANSACTION_IDLE:
			goto reset;
			break;
		
		// write register
		case TWI_TRANSACTION_WRITE_REG:
			if ((!(TWI.MASTER.STATUS & TWI_MASTER_WIF_bm)) ||
				(TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm)))
				goto reset;
			
			switch(state_AT)
			{
				case TWI_STATE_REGISTER:
					TWI.MASTER.DATA = reg_address_AT;
					state_AT = TWI_STATE_DATA;
					break;
				
				case TWI_STATE_DATA:
					if (data_bytes_AT)
						TWI.MASTER.DATA = *data_buffer_AT++;
					else
						transaction_result_AT = TWI_RESULT_OK;
					break;
				
				default:
					goto reset;
			}
			break;
		
		// read register
		case TWI_TRANSACTION_READ_REG:
			if ((!(TWI.MASTER.STATUS & TWI_MASTER_WIF_bm)) ||
				(TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm)))
				goto reset;
			
			switch(state_AT)
			{
				case TWI_STATE_REGISTER:
					TWI.MASTER.DATA = reg_address_AT;
					state_AT = TWI_STATE_RESTART;
					break;
				
				case TWI_STATE_RESTART:
					TWI.MASTER.ADDR = (device_address_AT << 1) | TWI_READ_bm;		// sends repeated start and address
					state_AT = TWI_STATE_DATA;
					transaction_type_AT = TWI_TRANSACTION_READ;						// common code
					break;

				default:
					goto reset;
			}
			break;

		// read
		case TWI_TRANSACTION_READ:
			if ((!(TWI.MASTER.STATUS & TWI_MASTER_RIF_bm)) ||
				(TWI.MASTER.STATUS & (TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm)))
				goto reset;
			
			switch(state_AT)
			{
				case TWI_STATE_DATA:
					data_bytes_AT--;
					if (data_bytes_AT != 0)
						TWI.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;							// send ACK
					else
					{
						TWI.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;		// send NACK
						transaction_result_AT = TWI_RESULT_OK;
					}
					*data_buffer_AT++ = TWI.MASTER.DATA;
					break;
				
				default:
					goto reset;
			}
			
	}
	
reset:
	TWI.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
	transaction_type_AT = TWI_TRANSACTION_IDLE;
	transaction_result_AT = TWI_RESULT_FAILED;
}

/**************************************************************************************************
* Write a register
*/
bool TWI_write_reg(uint8_t address, uint8_t reg, const uint8_t *buffer, uint8_t buffer_size)
{
	uint8_t	i;

	// wait for bus to become idle
	i = 0;
	while ((TWI.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) != TWI_MASTER_BUSSTATE_IDLE_gc)
	{
		if (i > TWI_IDLE_TIMEOUT_MS)
			return false;
		_delay_ms(1);
		i++;
	}

	// start write
	TWI.MASTER.CTRLA &= ~(TWI_MASTER_INTLVL_gm);		// make updates atomic
	transaction_type_AT = TWI_TRANSACTION_WRITE_REG;
	transaction_result_AT = TWI_RESULT_PENDING;
	state_AT = TWI_STATE_REGISTER;
	device_address_AT = address;
	reg_address_AT = reg;
	data_bytes_AT = buffer_size;
	data_buffer_AT = (uint8_t *)buffer;
	TWI.MASTER.CTRLA |= TWI_MASTER_INTLVL_gc;

	// start write
	TWI.MASTER.ADDR = address << 1;		// sends start and address and start interrupt state machine
	return true;
}

/**************************************************************************************************
* Read a register from TWI device
*/
bool TWI_read_reg(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t buffer_size)
{
	uint8_t	i;
				
	// wait for bus to become idle
	i = 0;
	while ((TWI.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) != TWI_MASTER_BUSSTATE_IDLE_gc)
	{
		if (i > TWI_IDLE_TIMEOUT_MS)
			return false;
		_delay_ms(1);
		i++;
	}

	// start read
	TWI.MASTER.CTRLA &= ~(TWI_MASTER_INTLVL_gm);		// make updates atomic
	transaction_type_AT = TWI_TRANSACTION_READ_REG;
	transaction_result_AT = TWI_RESULT_PENDING;
	state_AT = TWI_STATE_REGISTER;
	device_address_AT = address;
	reg_address_AT = reg;
	data_bytes_AT = buffer_size;
	data_buffer_AT = buffer;
	TWI.MASTER.CTRLA |= TWI_MASTER_INTLVL_gc;

	TWI.MASTER.ADDR = address << 1;		// sends start and address, starts interrupt state machine
	return true;
}

/**************************************************************************************************
* Read from TWI device
*/
bool TWI_read(uint8_t address, uint8_t *buffer, uint8_t buffer_size)
{
	uint8_t	i;

	// wait for bus to become idle
	i = 0;
	while ((TWI.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) != TWI_MASTER_BUSSTATE_IDLE_gc)
	{
		if (i > 10)
			return false;
		_delay_ms(1);
		i++;
	}

	// start read
	TWI.MASTER.CTRLA &= ~(TWI_MASTER_INTLVL_gm);		// make updates atomic
	transaction_type_AT = TWI_TRANSACTION_READ;
	transaction_result_AT = TWI_RESULT_PENDING;
	state_AT = TWI_STATE_DATA;
	device_address_AT = address;
	reg_address_AT = 0;
	data_bytes_AT = buffer_size;
	data_buffer_AT = buffer;
	TWI.MASTER.CTRLA |= TWI_MASTER_INTLVL_gc;

	TWI.MASTER.ADDR = (address << 1) | TWI_READ_bm;		// sends start and address
	return(true);
}


#endif


/**************************************************************************************************
 * Scan for TWI devices, output to terminal
 */

//this function is currently disabled. Need to change NULL to something else.
/*
int TWI_scan(void)
{
	uint8_t	i;
	
	for (i = 1; i < 127; i++)
	{
		if (TWI_read(i, NULL, 0))
			//printf_P(PSTR("Device found at 0x%02X\r\n"), i);
			return i;
	}
	return 0;
	//puts_P(PSTR("Scan complete.\r\n"));
}
*/