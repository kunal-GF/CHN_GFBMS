/*
 * uart.h
 *
 * Created: 10/23/2018 1:16:16 PM
 *  Author: User
 */ 

#include <avr/io.h>
#ifndef UART_H_
#define UART_H_





#endif /* UART_H_ */


void USART_Init( unsigned int ubrr);
void USART_Transmit( unsigned char data );
unsigned char USART_Receive( void );
