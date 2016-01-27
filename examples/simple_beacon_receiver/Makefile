.PHONY: %.upload
LIB = ../../lib/nrf24.c ../../lib/spi.c


upload: main.upload

clean:
	rm -f main.hex main.elf
%.elf: %.c
	avr-gcc -Wall -Werror -mmcu=atmega328p -Os -o $@ -I ../../lib/ $(LIB) uart.c $<

%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex $< $@

%.upload: %.hex
	avrdude -B 5 -c usbasp -p m328p -U flash:w:$<
