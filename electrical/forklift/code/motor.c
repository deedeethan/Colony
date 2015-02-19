#include <avr/io.h>
#include "motor.h"
#include <stdlib.h>

/*

motor controller

IN1: PD7
IN2: PD6
PWM: PB1


*/

int8_t motor_speed;

void motor_init(void)
{
  // WGM1 0b0101 (fast PWM, 8-bit)
  // COM1A 0b10 (clear OC1A on compare match, set on BOTTOM)
  // CS1 0b011 (64 prescaler)
  TCCR1A = _BV(WGM10) | _BV(COM1A1);
  TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10);
  OCR1AH = 0;
  
  // set IN1 (PD7), IN2 (PD6), and PWM (PB1) as output
  DDRD |= _BV(DDD7) | _BV(DDD6);
  DDRB |= _BV(DDB1);

  set_motor(0);
}

void set_motor(int8_t speed)
/* Speed must be between -127 and 127 */
{
  motor_speed = speed;
  OCR1AL = (uint8_t)abs(speed)*2;
  if(speed>0) //go forwards
  {
    PORTD |= (1<<PD7);
    PORTD = PORTD & ~(1<<PD6);
  }  
  if(speed<0) //go backwards
  {
    PORTD |= (1<<PD6);
    PORTD = PORTD & ~(1<<PD7);
  }
  if(speed==0) // turn motor off
  {
    PORTD = PORTD & ~(1<<PD6);
    PORTD = PORTD & ~(1<<PD7);
  }
}

int8_t get_motor()
{
  return motor_speed;
}
