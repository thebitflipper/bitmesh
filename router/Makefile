.PHONY: %.upload
LIB = ../lib/nrf24.c ../lib/spi.c ../lib/uart.c ../lib/mesh.c ../lib/systick.c
MCU=atmega328p
LDFLAGS=-gc-sections,--print-gc-sections
CFLAGS=-Os -ffunction-sections -Wall -Werror
OBJECTS=$(patsubst %.c,%.o,$(LIB))
upload: sink.upload

clean:
	rm -f *.hex *.elf ../lib/*.o
%.o: %.c
	avr-gcc $(CFLAGS) -mmcu=$(MCU) -o $@ -I ../lib/ -c $<
%.elf: %.o $(OBJECTS)
	avr-gcc -s $(CFLAGS) -Wl,$(LDFLAGS) -mmcu=$(MCU) -o $@ -I ../lib/ $(OBJECTS) $<

%.size: %.elf
	file $<
	avr-size -A -d $<

%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex $< $@

%.upload: %.hex
	avrdude -B 0.5 -c usbasp -p m328p -U flash:w:$<

fuses:
	avrdude -v -B 5 -c usbasp -p m328p -U lfuse:w:0xE2:m -U hfuse:w:0xd9:m -U efuse:w:0x07:m
