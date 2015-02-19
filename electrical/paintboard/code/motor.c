#include <avr/io.h>
#include "motor.h"
#include <stdlib.h>

/*

motor controller. Timer2 is set to phase-correct PWM mode, with 32 prescaler.

AIN1: PC0
AIN2: PC1
PWMA: PB3 (Timer2) (OCRA)

BIN1: PC2
BIN2: PC3
PWMB: PD3 (Timer2) (OCRB)

*/

int8_t motor1_speed;
int8_t motor2_speed;

inline uint8_t abs_clamp(int8_t n)
{
  if (n == -128)
    return 127;
  else if (n < 0)
    return -n;
  else
    return n;
}

void motor_init(void)
{

  // WGM2 0b001 (phase-correct PWM, 8-bit)
  // COM2A 0b10 (clear OCR2A on compare match, set on BOTTOM)
  // COM2B 0b10 (clear OCR2B on compare match, set on BOTTOM)
  // CS2 0b011 (32 prescaler)
  TCCR2A = _BV(WGM20) | _BV(COM2A1) |_BV(COM2B1);
  TCCR2B = _BV(CS21) | _BV(CS20);
  OCR2A = 0;
  OCR2B = 0;
  
  // set output pins
  DDRC |= _BV(PC0) | _BV(PC1) | _BV(PC2) | _BV(PC3);
  DDRB |= _BV(PB3);
  DDRD |= _BV(PD3);

  set_motor1(0);
  set_motor2(0);
}

void set_motor1(int8_t speed)
/* Speed must be between -127 and 127 */
{
  motor1_speed = speed;
  OCR2A = abs_clamp(speed)*2;
  if(speed>0) //go forwards
  {
    PORTC |= (1<<PC0);
    PORTC &= ~(1<<PC1);
    PORTC = PORTC & ~(1<<PC1);
  }  
  if(speed<0) //go backwards
  {
    PORTC &= ~(1<<PC0);
    PORTC |= (1<<PC1);
    PORTC = PORTC & ~(1<<PC0);
  }
  if(speed==0) // turn motor off
  {
    PORTC = PORTC & ~(1<<PC1);
    PORTC = PORTC & ~(1<<PC0);
  }
}

void set_motor2(int8_t speed)
/* Speed must be between -127 and 127 */
{
  motor2_speed = speed;
  OCR2B = abs_clamp(speed)*2;
  if(speed>0) //go forwards
  {
    PORTC |= (1<<PC2);
    PORTC &= ~(1<<PC3);
    PORTC = PORTC & ~(1<<PC3);
  }  
  if(speed<0) //go backwards
  {
    PORTC &= ~(1<<PC2);
    PORTC |= (1<<PC3);
    PORTC = PORTC & ~(1<<PC2);
  }
  if(speed==0) // turn motor off
  {
    PORTC = PORTC & ~(1<<PC3);
    PORTC = PORTC & ~(1<<PC2);
  }
}

int8_t get_motor1()
{
  return motor1_speed;
}

int8_t get_motor2()
{
  return motor2_speed;
}
