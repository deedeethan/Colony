extern "C"
{
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
}
#include "range.h"

/* Ultrasonic Sensor:
 * -if RX pin is left open, it will continuously take readings
 * -PW output is 147us/in.
 * -PW will be high for a maximum of 37.5ms if no target is detected
 * 
 * 37.5ms * 8 MHz / 8 prescaler = 37500 max wait
 * 37.5ms * 16 MHz / 8 prescaler = problem
 * 37.5ms * 16 MHz / 64 prescaler = 9375 max wait
 * 147us/in * 16 MHz / 64 prescaler = 1.44685039 ticks / mm
 */

struct range_t {
  unsigned int start; // timer value on rising edge
  unsigned int value; // last measured range
  char busy;
} volatile range[2];

static void on_edge(int which)
{
  unsigned char int_high;
  unsigned int time = TCNT5;
  
  if (which)
  {
    int_high = PIN_SONAR_PWM & _BV(P_SONAR_PWM1);
  }
  else
  {
    int_high = PIN_SONAR_PWM & _BV(P_SONAR_PWM0);
  }
  
  if (int_high)
  {
    range[which].start = time;
    range[which].busy = 1;
  }
  else
  {
    // if timer overflowed since start, this arithmetic should still work out
    range[which].value = time - range[which].start;
    range[which].busy = 0;
  }
}

ISR(INT3_vect)
{
  on_edge(0);
}

ISR(INT2_vect)
{
  on_edge(1);
}

void range_init()
{
  // ISCx = 1, edge triggered
  EICRA |= _BV(ISC20) | _BV(ISC30);
  // enable INT2 and INT3
  EIMSK |= _BV(INT2) | _BV(INT3);
  
  // CS1 = 3, 1/64 prescaler
  // if this is changed, remember to change recv_edge in bom.cpp!
  TCCR5B = _BV(CS50) | _BV(CS51);

  // set tx as output
  DDRG |= _BV(DDG1);
  PORT_SONAR_TX &= ~ _BV(P_SONAR_TX);
}

void range_measure(unsigned int *values)
{
  int i;

  for (i = 0; i < 2; i++)
  {
    range[i].value = RANGE_ERR;
    range[i].busy = 0;
  }

  // TODO ensure that one interrupt won't be delayed because of the other
  PORT_SONAR_TX |= _BV(P_SONAR_TX);
  _delay_ms(40);
  PORT_SONAR_TX &= ~ _BV(P_SONAR_TX);

  for (i = 0; i < 2; i++)
  {
    while (range[i].busy) {}
    values[i] = range[i].value;
  }
}
