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

#define CONFIG	    0x00
#define EN_AA	    0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH	    0x05
#define RF_SETUP    0x06
#define STATUS	    0x07
#define OBSERVE_TX  0x08
#define RPD	    0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR	    0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0

#define RX_DR       6


#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define W_TX_PAYLOAD 0xA0
#define R_RX_PAYLOAD 0x61

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

void NRF24_set_register(uint8_t reg, uint8_t val){
	CSN_lo;
	SPI_Transmit(W_REGISTER | (REGISTER_MASK & reg));
	SPI_Transmit(val);
      	CSN_hi;
}

void NRF24_flush_tx(){
	CSN_lo;
	SPI_Transmit(FLUSH_TX);
      	CSN_hi;
}
void NRF24_flush_rx(){
	CSN_lo;
	SPI_Transmit(FLUSH_RX);
      	CSN_hi;
}

void NRF24_set_payload(uint8_t *payload, uint8_t length){
	CSN_lo;
	SPI_Transmit(W_TX_PAYLOAD);
	for(uint8_t i = 0; i < length; i++){
		SPI_Transmit(payload[i]);
	}
      	CSN_hi;
}
uint8_t NRF24_read_status(){
	CSN_lo;
	uint8_t s = SPI_Transmit(0xff);
	CSN_hi;
	return s;
}

void NRF24_read_payload(uint8_t *payload){
	CSN_lo;
	SPI_Transmit(R_RX_PAYLOAD);
	for(uint8_t i = 0; i < 16; i++){
		payload[i] = SPI_Transmit(0xff);
	}
      	CSN_hi;
}
void NRF24_set_channel(uint8_t chan){
	if(chan > 125){
		chan = 125;
	}
	NRF24_set_register(RF_CH, chan);
}

void NRF24_set_low_speed(){
	NRF24_set_register(RF_SETUP, 0x26);
}
void NRF24_set_hi_speed(){
	NRF24_set_register(RF_SETUP, 0x06);
}

int main (void) {
	init();

	uint8_t r =(1 << MASK_RX_DR  )| /* No rx interrupts */
		(1 << MASK_TX_DS  )| /* No tx interrupts */
		(1 << MASK_MAX_RT )| /* No rt interrupts */
		(1 << EN_CRC      )| /* Enable crc */
		(0 << CRCO        )| /* 1 byte crc */
		(1 << PWR_UP      )| /* Power up */
		(1 << PRIM_RX     ); /* PRX mode */
	NRF24_set_register(CONFIG, r);
	NRF24_set_register(RX_PW_P0, 16);
	NRF24_flush_tx();
	NRF24_flush_rx();
	NRF24_set_channel(120);
	NRF24_set_low_speed();

	uint8_t payload[16];
	printf("Init\n");
	CE_hi;
	while (1) {
		/* Check if there is any data in rx pipe 0 */
		if((NRF24_read_status() & 0x0E) != 0x0E){
			printf("Data -> ");
			/* Fetch the payload */
			NRF24_read_payload(payload);
			for (uint8_t i = 0; i < 16; i++){
				printf("%02hhX", payload[i]);
			}
			printf("\n");
			/* Reset the status register */
			NRF24_set_register(STATUS, 0x70);
		}
	}
}
