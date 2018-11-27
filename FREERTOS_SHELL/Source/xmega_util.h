/*
 * xmega_util.h
 *
 * Created: 9/16/2018 7:58:39 PM
 *  Author: Charlie
 */ 


#ifndef XMEGA_UTIL_H_
#define XMEGA_UTIL_H_

#include <avr/io.h>
#include <avr/interrupt.h>

void CCPWrite( volatile uint8_t * address, uint8_t value );
void config_SYSCLOCK();



#endif /* XMEGA_UTIL_H_ */