#ifndef __includes_h
#define __includes_h

/* Make flycheck more happy */
#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#include <avr/io.h>
#include  <avr/pgmspace.h>
/* Internal */
#define F_CPU 8000000UL
#include <util/delay.h>
#include <stdlib.h>

#define ADDR {0x8E, 0x89, 0xBE, 0xD6, 0x00}

#define NRF24_PORT PORTB
#define NRF24_DDR  DDRB
#define CE	 PB1
#define CSN	 PB2

#define DEBUG

#ifdef DEBUG
#define D(A,...) printf_P(PSTR("D(%02d) %15s(%3d) " A), mesh.addr, __FILE__, __LINE__, ##__VA_ARGS__);
#else
#define D(A,...) { };
#endif

#endif
