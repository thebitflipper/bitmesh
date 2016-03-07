#define F_CPU 8000000UL
#include <includes.h>
#include <mesh.h>
#include <uart.h>
#include <nrf24.h>
#include <systick.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>

#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)
#define toggle_output(portdir,pin) portdir ^= (1<<pin)


void as(uint8_t reg, uint8_t expect){
	if(NRF24_get_register(reg) != expect){
		printf("0x%02hhX is -> 0x%02hhX\n", reg, NRF24_get_register(reg));
	}

}

void start_sleeping() {
	// clear various "reset" flags
	MCUSR = 0;
	// allow changes, disable reset, enable Watchdog interrupt
	cli();
	/* WDTCSR = _BV (WDCE) | _BV (WDE); */
	/* // set interval (see datasheet p55) */
	/* WDTCSR =1 << WDIE | */
	/* 	1 << WDP3 | */
	/* 	0 << WDP2 | */
	/* 	0 << WDP1 | */
	/* 	1 << WDP0; */
	sei();
	/* wdt_reset();  // start watchdog timer */
	set_sleep_mode (SLEEP_MODE_PWR_DOWN); // prepare for powerdown
	sleep_enable();

	// turn off brown-out enable in software
	MCUCR = _BV (BODS) | _BV (BODSE);
	MCUCR = _BV (BODS);

	/* previousADCSRA = ADCSRA; */
	ADCSRA &= ~(1<<ADEN); //Disable ADC
	ACSR = (1<<ACD); //Disable the analog comparator
	DIDR0 = 0x3F; //Disable digital input buffers on all ADC0-ADC5 pins
	DIDR1 = (1<<AIN1D)|(1<<AIN0D); //Disable digital input buffer on AIN1/0

	power_twi_disable();
	power_spi_disable();
	power_usart0_disable(); //Needed for serial.print
	power_timer0_disable(); //Needed for delay and millis()
	power_timer1_disable();
	power_timer2_disable(); //Needed for asynchronous 32kHz operation

	sleep_cpu ();   // power down !
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
	mesh_init(0);
	printf("INIT!\n");

        set_output(DDRB, PB0);

	/* unsigned long p_delay = systick; */
	for(;;){

                /* if(systick < 10000){ */
                mesh_poll(systick);
                if(mesh_is_connected()){
                        output_high(PORTB, PB0);
                        _delay_us(40);
                        output_low(PORTB, PB0);
                        _delay_ms(20);
                } else {
                        output_high(PORTB, PB0);
                        _delay_ms(20);
                        output_low(PORTB, PB0);
                }
                /* } else { */

                /*         NRF24_power_down(); */
                /*         start_sleeping(); */
                /* } */
		/* as(CONFIG,     0x0B); */
		/* as(EN_AA,      0x3F); */
		/* as(EN_RXADDR,  0x07); */
		/* as(SETUP_AW,   0x03); */
		/* as(SETUP_RETR, 0x35); */
		/* as(RF_CH,      0x78); */
		/* as(RF_SETUP,   0x27); */
		/* as(STATUS,     0x0E); */
		/* as(DYNPD,      0x00); */
		/* as(FEATURE,    0x01); */
		/* printf("test %d %lu\n", mesh_is_connected(), systick); */
		/* if((systick - p_delay) > 10000){ */
		/* 	NRF24_print_registers(); */
		/* 	p_delay = systick; */
		/* } */



	}
	return 0;
}
