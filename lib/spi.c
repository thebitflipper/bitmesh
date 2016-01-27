#include "spi.h"

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
