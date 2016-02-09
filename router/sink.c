
#define F_CPU 8000000UL
#include <includes.h>
#include <mesh.h>
#include <uart.h>
#include <nrf24.h>
#include <systick.h>

#include <util/delay.h>
void as(uint8_t reg, uint8_t expect){
	if(NRF24_get_register(reg) != expect){
		printf("0x%02hhX is -> 0x%02hhX\n", reg, NRF24_get_register(reg));
	}

}

int main() {
	/* Enable the change of clock prescaler */
	CLKPR = (1 << CLKPCE);
	/* Within 4 clock cycles change the prescaler */
	CLKPR = (0 << CLKPS0 |
		 0 << CLKPS1 |
		 0 << CLKPS2 |
		 0 << CLKPS3);

	uart_init();
	systick_init();
	mesh_init(1);
	printf("Sink init!\n");
	for(;;){
		mesh_poll(systick);
		if(NRF24_get_register(0x09)){
			printf(".");
		}
		/* as(CONFIG,     0x0B); */
		as(EN_AA,      0x3F);
		/* as(EN_RXADDR,  0x23); */
		/* as(SETUP_AW,   0x03); */
		/* as(SETUP_RETR, 0x35); */
		/* as(RF_CH,      0x78); */
		/* as(RF_SETUP,   0x27); */
		/* as(STATUS,     0x0E); */
		/* as(DYNPD,      0x00); */
		/* as(FEATURE,    0x01); */
		/* as(FIFO_STATUS,0x11); */
		/* as(RX_PW_P5,   0x10); */
		/* as(RX_ADDR_P5, 0xff); */


		/* printf("test %d %lu\n", mesh_is_connected(), systick); */
		_delay_ms(100);
	}
	return 0;
}
