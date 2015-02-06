#PART=m328p
#MCU=atmega328

PART=t85
MCU=attiny85

PROG=avrispMKII
TARGET ?= bom.hex

F_CPU=1000000

SRC=bom.c tiny-twi.c
HDR=
FLAGS=-mmcu=$(MCU) -DF_CPU=$(F_CPU)UL -funsigned-char -Os -fpack-struct -Wall
BOOT_FLAGS=$(FLAGS) -Wl,-Ttext=0xA00,--defsym=strt_pgrm=0x1E
VECT_FLAGS=$(FLAGS) -nostartfiles

default: bom.hex bootloader.hex bootvectors.hex

%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex $< $@

%.bin: %.elf
	avr-objcopy -j .text -j .data -O binary $< $@

bom.elf: $(SRC) $(HDR)
	avr-gcc $(FLAGS) $(SRC) -o $@

%.s: %.c
	avr-gcc $(FLAGS) -S $< -o $@

program: $(TARGET)
	avrdude -p $(PART) -c $(PROG) -P usb -B 5 -U flash:w:$(TARGET)

blinky.elf: blinky.c
	avr-gcc $(FLAGS) blinky.c -o $@

bootloader.elf: bootloader.c tiny-twi-sync.c tiny-twi-sync.h
	avr-gcc $(BOOT_FLAGS) bootloader.c tiny-twi-sync.c -o $@

bootvectors.elf: bootvectors.S
	avr-gcc $(VECT_FLAGS) bootvectors.S -o $@

burn: bootloader.hex bootvectors.hex
	avrdude -p $(PART) -c $(PROG) -P usb -B 5 -U flash:w:bootloader.hex \
	  -U flash:w:bootvectors.hex
