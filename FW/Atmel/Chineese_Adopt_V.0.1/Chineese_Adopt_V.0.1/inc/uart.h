/*
 * uart.h
 *
 * Created: 10/23/2018 1:16:16 PM
 *  Author: User
 */ 

#include <avr/io.h>
#ifndef UART_H_
#define UART_H_
#define FOSC 8000000 // Clock Speed
#define MYUBRR FOSC/16/BAUD-1
#endif /* UART_H_ */


void USART_Init( unsigned int baud);
void UART_SendData8(uint8_t data);
void TU_send(uint8_t data);
void TU_puts(uint8_t *array);
void TU_putln(uint8_t *array);

uint8_t UART_ReceiveData8(void);
uint8_t TU_getc(uint8_t *data);

void TU_putHex(uint8_t data);
