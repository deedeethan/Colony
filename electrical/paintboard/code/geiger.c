#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "geiger.h"


int16_t count;
int16_t rate;
int16_t ticks;

void geiger_init()
{
  EICRA |= _BV(ISC11) | _BV(ISC10);
  EIMSK |= _BV(INT1);
}

void geiger_tick()
{
  ticks++;
  if(ticks==100)
  {
    ticks=0;
    rate = count*3;
    count = 0;
  }
}

int16_t geiger_rate()
{
  return (rate==0)?count:rate;
}

ISR (INT1_vect)
{
  count++;
}
