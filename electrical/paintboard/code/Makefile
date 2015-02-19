SRC = main.c twi.c motor.c servo.c sol.c geiger.c
HDR = twi.h motor.h servo.h sol.h geiger.h

paint: paint.hex
	@

%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex $< $@

paint.elf: $(SRC) $(HDR)
	avr-gcc -mmcu=atmega328 -DF_CPU=8000000UL $(SRC) -o paint.elf -Os

program: paint.hex
	avrdude -c avrispmkII -p m328 -P usb -B 1 -U flash:w:paint.hex

clean:
	rm -f paint.elf paint.hex
