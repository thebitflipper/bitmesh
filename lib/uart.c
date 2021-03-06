#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "uart.h"
#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#ifndef BAUD
#define BAUD 250000
#endif
#include <util/setbaud.h>

#define BUF_SIZE 16

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

void uart_init(void) {
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;

#if USE_2X
	UCSR0A |= _BV(U2X0);
#else
	UCSR0A &= ~(_BV(U2X0));
#endif

	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
	UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);   /* Enable RX and TX */
	sei();
	stdout = &uart_output;
	stdin  = &uart_input;
}

volatile uint8_t buffer[BUF_SIZE];
volatile uint8_t buffer_pos = 0;

ISR(USART_RX_vect){
	buffer[buffer_pos] = UDR0;
	if(buffer_pos < BUF_SIZE){
		buffer_pos++;
	}
}

int uart_putchar(char c, FILE *stream) {
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
	return 1;
}

int uart_getchar(FILE *stream) {
	loop_until_bit_is_set(UCSR0A, RXC0);
	return UDR0;
}
