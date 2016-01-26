.PHONY: %.upload
upload: main.upload
clean:
	rm -f main.hex main.elf
%.elf: %.c
	avr-gcc --std=c99 -Wall -Werror -mmcu=atmega328p -Os -o $@ uart.c $<

%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex $< $@

%.upload: %.hex
	avrdude -B 5 -c usbasp -p m328p -U flash:w:$<
