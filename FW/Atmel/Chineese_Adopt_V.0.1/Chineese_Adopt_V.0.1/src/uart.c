/*
 * uart.c
 *
 * Created: 10/23/2018 1:17:14 PM
 *  Author: Pankaj Kumar
 */ 


#include <avr/io.h>
#include "uart.h"

void USART_Init( unsigned int baud)
{
	volatile uint8_t ubbr;
	ubbr = FOSC/16/baud-1;
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubbr>>8);
	UBRR0L = (unsigned char)ubbr;
	/*Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}


void UART_SendData8(uint8_t data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}


/* Send 1 byte of Data */
void TU_send(uint8_t data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	UART_SendData8(data);
}

/* Sends out a NULL terminated string of data */
void TU_puts(uint8_t *array)
{
	uint8_t *p = array;
	while(*p)
	{
		/* Wait for empty transmit buffer */
		while ( !( UCSR0A & (1<<UDRE0)) );
		UART_SendData8(*p);
		++p;
	}
}

/* Send out a NULL terminated string of data with Line Terminator */
void TU_putln(uint8_t *array)
{
	TU_puts(array);
	TU_puts("\r\n");
}





/**
  * @brief  Returns the most recent received data by the UART2 peripheral.
  * @param  None
  * @retrieval Received Data
  */
uint8_t UART_ReceiveData8(void)
{
  return UDR0;
}


/* Detects if any things is available in the internal buffer 
   then received it in a pointed location 
	 If there is nothing then 0 is returned 
*/
uint8_t TU_getc(uint8_t *data)
{
	if(UDR0)
	{
		*data = UART_ReceiveData8();
		return 1;
	}
	return 0;
}


/* Send one byte Hex with '0x' prefix on the serial port */
void TU_putHex(uint8_t data)
{
	uint8_t tmp;
	TU_puts("0x");
	tmp = (data >> 4) & 0x0F;
	tmp = (tmp > 9)?('A' + tmp - 10):('0' + tmp);
	TU_send(tmp);
	tmp = data & 0x0F;
	tmp = (tmp > 9)?('A' + tmp - 10):('0' + tmp);
	TU_send(tmp);
}