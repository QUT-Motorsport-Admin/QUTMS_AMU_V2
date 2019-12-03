/*****************************************************************************
* @file    firmware/QUTMS_SteeringWheel_Firmware/UART.c
* @author  Zoe Goodward
* @version V1.0.0
* @date    4/09/2019 5:15:38 PM
* @brief   This file...
*****************************************************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "UART.h"

/* All boards use a 16MHz oscillator frequency */
// 
void uart0_init(unsigned int baudRate) {
	/* Set baud rate 19200 */
	UBRR0H = 0; //1001011
	UBRR0L = 103; 
	CLKPR = (1<<CLKPCE);  	//enable clock prescaler change
	CLKPR = 0x00;			//clock divide = 1  so  16MHz/1 = 16MHz
	/* Enable receiver and transmitter */
	UCSR0B = (1 << RXEN0)|(1 << TXEN0);
	/* Set frame format: 8data, 0stop bit */
	UCSR0A = 2;		//2x on
	UCSR0C = (3 << UCSZ00);
}

void uart0_transmit(uint8_t data) {
	/* wait for empty transmit buffer */
	while( !(UCSR0A & (1 << UDRE0)) );
	/* put data into buffer, sends the data */
	UDR0 = data;
}

uint8_t uart0_receive(void) {
	/* wait for data to be received */
	while( !(UCSR0A & (1 << RXC0)) );
	/* get and return received data from buffer */
	return UDR0;
}

/* Flush the receive buffer */
void uart0_flush(void) {
	uint8_t dummy;
	while( UCSR0A & (1 << RXC0) ) {
		dummy = UDR0;
	}
}

//PRUSART0 must write to 0

// double speed U2Xn in UCSRnA to 1

//For interrupt driven USART operation, the Global Interrupt Flag should be cleared 
// (and interrupts globally disabled) when doing the initialization

//void uart_init(unsigned int baudrate)
//{
	//UART_TxHead = 0;
	//UART_TxTail = 0;
	//UART_RxHead = 0;
	//UART_RxTail = 0;
	//
	//CLKPR = (1<<CLKPCE);  	//enable clock prescaler change
	//CLKPR = 0x00;			//clock divide = 1  so  16MHz/1 = 16MHz
//
	//UBRR0H = 0;
	//UBRR0L = 103;	//19200 baud for laptop/LCD info panel
//
	///* Set frame format: asynchronous, 8data, no parity, 1stop bit */
	//UCSR0A = 2;		//2x on
	//UCSR0B = 24;	//tx and rx on  AND rxie on for testing
	//UCSR0C = 6;		//8 - 1 - N
//
	//}/* uart_init */