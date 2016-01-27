#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif
#define F_CPU 8000000UL

#include <avr/io.h>
#include <stdio.h>
#include "uart.h"
#include <util/delay.h>
#include <string.h>
#include <nrf24.h>
#include <spi.h>
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
}


int main (void) {
	init();
	NRF24_init(0, 1);
	NRF24_set_pl_len(0, 16);
	NRF24_set_channel(120);
	NRF24_set_speed(NRF_250KBIT);
	uint8_t payload[16] = {0};
	printf("Init\n");
	while (1) {
		NRF24_poll_handle();
		if(NRF24_read_payload(payload, NULL, NULL)){
			printf("Data  -> ");
			for (uint8_t i = 0; i < 16; i++){
				printf("%02hhX", payload[i]);
			}
			printf("\n");
		}
	}
}
