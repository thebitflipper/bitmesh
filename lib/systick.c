#include "systick.h"
#include "includes.h"
#include <avr/interrupt.h>

volatile unsigned long systick;

ISR(TIMER1_COMPA_vect) {			//tmr1 CTC / systick tmr
	systick++;
}

void systick_config(unsigned short duration) {
   	TCCR1A =(0<<COM1A1) | (0<<COM1A0) | //normal operation for com1a
		(0<<COM1B1) | (0<<COM1B0) | //normal operation for channel b
		(0<<WGM11) | (0<<WGM10); //wgm13..0=0b0100, CTC mode
	TCCR1B =(0<<ICNC1) | //input caputure noise canceller: disabled
		(0<<ICES1) | //input caputure edge select: falling edge
		(0<<WGM13) | (1<<WGM12) | //CTC mode
		(0<<CS12) | (1<<CS11) | (0<<CS10);	//cs2..0=0b001, timer activated, 1:1 prescaler
 	TCCR1C =(0<<FOC1A) | //channel a output disabled
		(0<<FOC1B); //channel b output disabled
   	OCR1A = duration; //set the top of CTC on channel a
	TCNT1 = 0; //reset timer counter
	TIMSK1 |= (1<<OCIE1A); //output compare interrupt for channel a enabled
	sei(); //enable interrupt
}

void systick_init(void) {
	systick_config(1000);
}
