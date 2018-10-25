/*
 * AFE_init.c
 *
 * Created: 10/25/2018 10:16:48 AM
 *  Author: User
 */ 


#include <avr/io.h>
#include "AFE_init.h"



void afe_init(void)
{
	DDRB = 0xFF; 
	DDRC = 0xFF; 
	
	PORTB &= ~(1<<PINB0);
	PORTC |= 1<<PINC0;
}