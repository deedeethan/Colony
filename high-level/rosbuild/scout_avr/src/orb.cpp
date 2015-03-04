extern "C" {
#include <avr/io.h>
#include <util/delay.h>
}
#include "orb.h"

void orb_init() {

  /* set pins to output mode */
	DDRB |= _BV(PB4) | _BV(PB5) | _BV(PB6);
  DDRE |= _BV(PE3) | _BV(PE4) | _BV(PE5);

  orb_set0(0, 0, 0);
  //orb_set1(0, 0, 0);
  orb_set1(0, 255, 255);
  	
	/* timer 2 */
  TCCR2A |= _BV(COM2A1) | _BV(COM2A0) | _BV(WGM20);
	TCCR2B |= _BV(CS21);

	/* timer 1 */
	TCCR1A |= _BV(COM1A1) | _BV(COM1A0) |  _BV(COM1B1) | _BV(COM1B0) |_BV(WGM10);
	TCCR1B |= _BV(CS11);

	/* timer 3 */
  TCCR3A |= _BV(COM3A1) | _BV(COM3A0) | _BV(COM3B1) | _BV(COM3B0) | _BV(COM3C1)
    | _BV(COM3C0) | _BV(WGM30);
	TCCR3B |= _BV(CS31);
}

void orb_set0(unsigned char r, unsigned char g, unsigned char b) {
	OCR1B = r;
	OCR2A = g;
	OCR1A = b;
}

void orb_set1(unsigned char r, unsigned char g, unsigned char b) {
	OCR3B = r;
	OCR3C = g;
	OCR3A = b;
}
