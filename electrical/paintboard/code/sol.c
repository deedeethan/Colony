#include "sol.h"
#include <avr/io.h>

void sol_init() {
  DDRD |= _BV(DDD7);
  DDRB |= _BV(DDB7) | _BV(DDB6) | _BV(DDB0);
}

void set_sol1(char on) {
  if (on)
    PORTD |= _BV(PD7);
  else
    PORTD &= ~_BV(PD7);
}

void set_sol2(char on) {
  if (on)
    PORTB |= _BV(PB0);
  else
    PORTB &= ~_BV(PB0);
}

void set_sol3(char on) {
  if (on)
    PORTB |= _BV(PB6);
  else
    PORTB &= ~_BV(PB6);
}

void set_sol4(char on) {
  if (on)
    PORTB |= _BV(PB7);
  else
    PORTB &= ~_BV(PB7);
}
