/*
* cliffSensor.cpp
*
* Created: 2012-9-24 17:38:59
*  Author: Anson
*/

#include "cliffSensor.h"
#include <avr/io.h>
#include <avr/interrupt.h>
unsigned char flag_cliff0=0,flag_cliff1=0,flag_cliff2=0; // are these global variables???

void cliffSensor_init(void)
{
  sei();   //set the I-bit of SREG to 1, enabling global interrupters
	EIMSK |= 0b01000011; //enable INT0,INT1,INT6
	EICRA |= 0b00001111; 
	EICRB |= 0b00110000; //The rising edge of INTn triggers an interrupt request.
  DDRD &= ~(0b00000011);
  DDRE &= ~(0b01000000); // set PD0, PD1 and PE6 as input pins.
}

bool read_cliffSensor_front(void)
{
	if(flag_cliff0 || (PIND & (1<<PD0))) //either a rising edge or a high logic level will be determined as cliff
	{
		flag_cliff0 = 0; // reset the flag
		EIMSK |= (1<<INT0); // re-enable the interrupt
		return true;
	}
	return false;
}

bool read_cliffSensor_left(void)
{
	if(flag_cliff1 || (PIND & (1<<PD1)))
	{
		flag_cliff1 = 0;
		EIMSK |= (1<<INT1);
		return true;
	}
	return false;
}

bool read_cliffSensor_right(void)
{
	if(flag_cliff2 || (PINE & (1<<PE6)))
	{
		flag_cliff2 = 0;
		EIMSK |= (1<<INT6); // enable INT6
		return true;
	}
	return false;
}

unsigned char read_cliffSensor_all(void) // return a 3-digit binary number(left, front, right)
{
	unsigned char result=0;
	if(read_cliffSensor_left()){result |= 0b100;}
	if(read_cliffSensor_front()){result |= 0b010;}
	if(read_cliffSensor_right()){result |= 0b001;}
	return result;
}

ISR(INT0_vect)
{
	flag_cliff0 = 1;
	EIMSK &= ~(1<<INT0); // disable INT0
}

ISR(INT1_vect){
	flag_cliff1 = 1;
	EIMSK &= ~(1<<INT1); // disable INT1
}

ISR(INT6_vect){
	flag_cliff2 = 1;
	EIMSK &= ~(1<<INT6); // disable INT6
}
