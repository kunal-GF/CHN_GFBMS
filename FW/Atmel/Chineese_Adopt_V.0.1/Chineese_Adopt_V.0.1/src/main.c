/*
 * Chineese_Adopt_V.0.1.c
 *
 * Created: 10/22/2018 8:24:40 PM
 * Author : User
 */ 


#define FOSC 8000000 // Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>





int main(void)
{
	USART_Init(MYUBRR);
	
	while(1)
	{
		_delay_ms(1000);
		USART_Transmit(raw_data);
	}
}