SRC = forklift.c analog.c twi.c motor.c
HDR = analog.h twi.h motor.h

forklift: forklift.hex
	@

%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex $< $@

forklift.elf: $(SRC) $(HDR)
	avr-gcc -mmcu=atmega168 -DF_CPU=8000000L $(SRC) -o forklift.elf -Os

download: forklift.hex
	avrdude -c avrispmkII -p m168 -P usb -B 1 -U flash:w:forklift.hex

clean:
	rm -f forklift.elf forklift.hex
