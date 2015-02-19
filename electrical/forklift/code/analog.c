#include <avr/io.h>
#include "analog.h"

int line_threshold = 150;

void analog_init(void)
{
  // ADMUX register
  // Bit 7,6 - Set voltage reference to AVcc (0b01)
  // Bit 5 - ADLAR not set
  // Bit 4 - X
  // Bit 3:0 - Current channel
  ADMUX = _BV(REFS0);

  // ADC Status Register A
  // Bit 7 - ADEN  analog enable set
  // Bit 6 - ADSC  start conversion bit not set
  // Bit 5 - ADATE enable auto trigger (for free running mode) not set
  // Bit 4 - ADIF  ADC interrupt flag
  // Bit 3 - ADIE  enable ADC interrupt (required for free-running mode)
  // Bits 2-0 - ADPS set to 8 MHz / 64 = 125 kHz (should be 50-200 kHz)
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1);

  // Set line sensor mux lines PD2, PD3, and PB3 (MOSI) to output
  DDRD |= _BV(DDD2) | _BV(DDD3);
  DDRB |= _BV(DDB3);
}

int analog_read(int which)
{
  ADMUX = (ADMUX & 0xF0) | (which & 0x0F);

  // Start the conversion
  ADCSRA |= _BV(ADSC);

  // Wait for the conversion to finish
  while (ADCSRA & _BV(ADSC));

  int adc_l = ADCL;
  int adc_h = ADCH;

  return ((adc_h << 8) | adc_l);
}

int line_read(int which)
{
  PORTD = (PORTD & 0xF3) | ((which & 3) << 2);
  PORTB = (PORTB & 0XF7) | ((which & 4) << 1);

  // For loop is used only as a delay to allow mux to settle
  volatile int i;
  for(i=0; i < 5; i++);
  
  return analog_read(ADC_LINE);
}

void line_update(char* values)
{
  int i;
  for(i = 0; i<5; i++)
    values[i] = line_read(i) < line_threshold ? LBLACK : LWHITE;
}

int line_locate(char* values)
{
  int i;
  int wsum = 0;
  int count = 0;

  for(i = 0; i < 5; i++)
  {
    count += values[i] / 2;
    wsum += i * values[i];
  }
  if (count == 0)
    return NOLINE;	
  if (count == 5)
    return FULL_LINE;
  return (wsum/count)-4;
}

int line_read_pos(void)
{
  char values[5];
  line_update(values);
  return line_locate(values);
}

void line_set_threshold_high(uint8_t threshold)
{
  line_threshold = ((int)(threshold & 3) << 8) | (line_threshold & 0xFF);
}

void line_set_threshold_low(uint8_t threshold)
{
  line_threshold = (line_threshold & 0x300) | (uint16_t) threshold;
}
