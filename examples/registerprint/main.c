/*
  ATmega328

  /Reset PC6|1   28|PC5
  PD0|2   27|PC4
  PD1|3   26|PC3
  PD2|4   25|PC2
  PD3|5   24|PC1
  PD4|6   23|PC0
  Vcc|7   22|Gnd
  Gnd|8   21|Aref
  PB6|9   20|AVcc
  PB7|10  19|PB5 SCK
  PD5|11  18|PB4 MISO
  PD6|12  17|PB3 MOSI
  PD7|13  16|PB2 CSN
  PB0|14  15|PB1
*/
#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif
#define F_CPU 8000000UL

#include <avr/io.h>
#include <stdio.h>
#include "uart.h"
#include <util/delay.h>
#include <string.h>
#define CE  PB1
#define CSN PB2
#define CSN_hi     PORTB |=  (1<<CSN);
#define CSN_lo     PORTB &= ~(1<<CSN);
#define CE_hi      PORTB |=  (1<<CE);
#define CE_lo      PORTB &= ~(1<<CE);

struct address {
	char name[12];
	char addr;
	unsigned char size;
};
struct address addresses[26] = {
	{"CONFIG",      0x00, 1},
	{"EN_AA",       0x01, 1},
	{"EN_RXADDR",   0x02, 1},
	{"SETUP_AW",    0x03, 1},
	{"SETUP_RETR",  0x04, 1},
	{"RF_CH",       0x05, 1},
	{"RF_SETUP",    0x06, 1},
	{"STATUS",      0x07, 1},
	{"OBSERVE_TX",  0x08, 1},
	{"RPD",	        0x09, 1},
	{"RX_ADDR_P0",  0x0A, 5},
	{"RX_ADDR_P1",  0x0B, 5},
	{"RX_ADDR_P2",  0x0C, 1},
	{"RX_ADDR_P3",  0x0D, 1},
	{"RX_ADDR_P4",  0x0E, 1},
	{"RX_ADDR_P5",  0x0F, 1},
	{"TX_ADDR",     0x10, 5},
	{"RX_PW_P0",    0x11, 1},
	{"RX_PW_P1",    0x12, 1},
	{"RX_PW_P2",    0x13, 1},
	{"RX_PW_P3",    0x14, 1},
	{"RX_PW_P4",    0x15, 1},
	{"RX_PW_P5",    0x16, 1},
	{"FIFO_STATUS", 0x17, 1},
	{"DYNPD",       0x1C, 1},
	{"FEATURE",     0x1D, 1},
};



void SPI_init(void){
	DDRB |= ((1<<PB3)|	/* MOSI */
		 (1<<PB5));	/* SCK  */

	SPCR = ((1<<SPE)|	/* SPI enable */
		(1<<MSTR)|	/* SPI master */
		(0<<SPR1)|	/* f_osc/4 */
		(0<<SPR0)|	/* f_osc/4 */
		(0<<CPOL)|	/* SCK idle low */
		(0<<CPHA));  	/* Sample on leading edge */

}

char SPI_Transmit(char cData){
	SPDR = cData;
	/* Wait for complete transfer */
	loop_until_bit_is_set(SPSR, SPIF);
	/* Return the received byte */
	return SPDR;
}
void init (void) {
	/* Enable the change of clock prescaler */
	CLKPR = (1 << CLKPCE);
	/* Within 4 clock cycles change the prescaler */
	CLKPR = (0 << CLKPS0 |
		 0 << CLKPS1 |
		 0 << CLKPS2 |
		 0 << CLKPS3);

	uart_init();
	/* Set stdout and stdin so that printf works */
	stdout = &uart_output;
	stdin  = &uart_input;

	/* Set the CE and CSN to output */
	DDRB |= (1 << CE);
	DDRB |= (1 << CSN);

	SPI_init();

	/* Default states */
	CSN_hi;
	CE_lo;

}


void printRegisters(){
	unsigned char input[5];
	/* Loop through all the registers */
	for (int i = 0; i < (int)(sizeof(addresses)/sizeof(addresses[0])); i++){
		/* Select the device */
		CSN_lo;
		/* Send the read command */
		SPI_Transmit(addresses[i].addr);
		/* Read all the bytes in that register */
		for(int ii = 0; ii < addresses[i].size; ii++){
			input[ii] = SPI_Transmit(0xff);
		}
		/* Unselect the device */
		CSN_hi;

		/* Print the register address, name, and content */
		printf("0x%02hhX %-11s 0x",addresses[i].addr, addresses[i].name);
		for(char ii = 0; ii < addresses[i].size; ii++){
			printf("%02hhX", input[0]);
		}
		putchar('\n');
	}

}


int main (void) {
	init();
	while (1) {
		printRegisters();
		_delay_ms(5000);
	}
}
