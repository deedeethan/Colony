
PART=m128rfa1
MCU=atmega128rfa1

#PART=m328p
#MCU=atmega328

#PART=m2560
#MCU=atmega2560

PROG=avrispMKII
#PROG=stk600

F_CPU=16000000
SRC=src/*.cpp src/ros_lib/*.cpp
HDR=src/*.h
FLAGS=-Isrc/ros_lib -Isrc -mmcu=$(MCU) -DF_CPU=$(F_CPU)UL -funsigned-char -Os -fpack-struct -Wall

ROS_MSG_DEPS=rosserial_msgs std_msgs messages

default: scout_avr.hex

# need the SRC globs above to be re-evaluated
all: ros_lib
	make scout_avr.hex

%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex $< $@

scout_avr.elf: $(SRC) $(HDR)
	avr-g++ $(FLAGS) $(SRC) -o scout_avr.elf

program: scout_avr.hex
	avrdude -p $(PART) -c $(PROG) -P usb -B 1 -U flash:w:scout_avr.hex

ros_lib: ros_lib_nomsgs msgs

ros_lib_nomsgs:
	rm -rf src/ros_lib
	cp -r `rospack find rosserial_client`/src/ros_lib src

msgs:
	rosrun rosserial_client make_library.py src $(ROS_MSG_DEPS)

clean:
	rm -rf scout_avr.elf scout_avr.hex src/ros_lib
