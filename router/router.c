#define F_CPU 8000000UL
#include <includes.h>
#include <mesh.h>
#include <uart.h>
#include <systick.h>
#include <util/delay.h>

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
	for(;;){
		mesh_poll(systick);
		printf("test %d %lu\n", mesh_is_connected(), systick);
		_delay_ms(100);
	}
	return 0;
}
