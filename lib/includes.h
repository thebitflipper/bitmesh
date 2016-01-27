#ifndef __includes_h
#define __includes_h

/* Make flycheck more happy */
#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#include <avr/io.h>

/* Internal */
#define F_CPU 8000000UL
#include <util/delay.h>
#include <stdlib.h>

#define NRF24_PORT PORTB
#define NRF24_DDR  DDRB
#define CE	 PB1
#define CSN	 PB2

#endif
