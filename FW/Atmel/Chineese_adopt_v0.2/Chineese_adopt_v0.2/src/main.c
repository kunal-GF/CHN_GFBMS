/*
 * Chineese_adopt_v0.2.c
 *
 * Created: 24-10-2018 11:31:40
 * Author : kunal
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
//#include "uart.h"
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

DDRB |= 0b0010000;

//Declaration of our functions
void USART_init(void);
uint8_t USART_receive(uint8_t *data);
void USART_send(uint8_t data);
void USART_putstring(char* StringPtr);
void TU_puts(uint8_t *array);
void TU_putln(uint8_t *array);
uint8_t TU_getln(uint8_t *array, uint8_t max_sz);


uint8_t String[20];    //String[] is in fact an array but when we put the text between the " " symbols the compiler threats it as a String and automatically puts the null termination character in the end of the text
uint8_t dt=20;
uint8_t dl;


int main(void)
{
	USART_init();        //Call the USART initialization code
	
	while(1)
	{        //Infinite loop
		//USART_putstring(String);    //Pass the string to the USART_putstring function and sends it over the serial
		       //Delay for 5 seconds so it will re-send the string every 5 seconds
		dl = TU_getln(String,dt);
		_delay_ms(1000);
		//USART_putstring(String);
		//_delay_ms(1000); 
		//USART_send(dt);
		//if(dl==1)
		//USART_send('s');
		//if(dl==0)
		//USART_send('f');
		//TU_puts("0x");
	}
	
	return 0;
}



void USART_init(void)
{
	
	UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (3<<UCSZ00);
}

uint8_t USART_receive(uint8_t *data)
{
	
	if((UCSR0A & (1<<RXC0)))
	{
		*data = UDR0;
		return 1;
	}
return 0;
	
	
}

void USART_send( uint8_t data)
{
	
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
	
}

void USART_putstring(char* StringPtr)
{
	
	while(*StringPtr != 0x00)
	{
		USART_send(*StringPtr);
		StringPtr++;
	}
	
}

void TU_puts(uint8_t *array)
{
	uint8_t *p = array;
	while(*p)
	{
		while(!(UCSR0A & (1<<UDRE0)));
		UDR0 = *p;
		++p;
	}
}

void TU_putln(uint8_t *array)
{
	TU_puts(array);
	TU_puts("\r\n");
}

uint8_t TU_getln(uint8_t *array, uint8_t max_sz)
{
	static uint8_t nCount = 0; // State Counter for Reception
	static uint8_t tm = 0; // Termination Marker 1-First Terminator 0-Nothing
	uint8_t dt, ret = 0;
	
	// Works only if there is some thing in the RX
	while(USART_receive(&dt))
	{
		switch(dt)
		{
			case '\r': // First Terminator Received
			tm = 1; // Alert that First terminator Received
			break;
			
			case '\n': // Second Terminator
			if(tm == 1) // Last terminator received
			{
				tm = 0; // Completed the sequence
				array[nCount] = 0; // Insert the NULL terminator
				if(nCount == 0) // For 1 character as NULL for only EOL Key
				{
					ret = 1;
				}
				else // In case we had bytes in the Array
				{
					ret = nCount; // Return Array size
					nCount = 0; // Reset the sequence
				}
			}
			else // Wrong terminator received
			{
				ret = 0;  // Clear Buffer Data
				nCount = 0;
				array[nCount] = 0; // Insert the NULL terminator
			}
			break;
			case '\b': // Backspace received
			if(nCount) // We were already in middle of receiving some thing
			{
				array[nCount] = 0; // Clear the Last Received
				--nCount; // Decrement the counter
			}
			else // Clear the Data as correctly we do not need backspace
			{
				dt = 0;
			}
			break;
			default:
			// Already receiving the Packet
			if(nCount)
			{
				if(nCount < (max_sz-1)) // Check for space in Buffer added for NULL
				{
					array[nCount++] = dt;
				}
				else // Buffer is Full need to Return
				{
					array[nCount] = 0; // Insert the NULL terminator
					ret = nCount; // Return Array size
					nCount = 0; // Reset the sequence
				}
			}
			else // Initiate the Process of Reception
			{
				array[nCount++] = dt;// Start the Reception
			}
			break;
		}// End of the Selector Switch(dt)
		
		// Send back the Character (If not Null)
		if(dt)
		{
		USART_send(dt);
		USART_send('y');	
		}
		
	}// End of While TU_getc
	
	return ret;
}