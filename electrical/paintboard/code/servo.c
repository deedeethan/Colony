#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "servo.h"

/*

Servo controller. Timer0 is set to fast PWM mode, with 64 prescaler. Outputs
are cleared on BOTTOM and set on compare match, and the compare values are set
to 255 minus the regular value. This way the timer can be stopped when it
overflows and the output will end up off, but will have been on for the correct
amount of time.

SERVO_PWM1: PD6 (Timer0)

SERVO_PWM2: PD5 (Timer0)

*/

#define PRESCALER 64
#define MS_TO_TICKS(x) ((uint16_t)((x) * (F_CPU / 1000 / PRESCALER)))

#define MIN_TICKS MS_TO_TICKS(1)
#define MAX_TICKS MS_TO_TICKS(2)

static uint8_t angle_to_ticks(int8_t angle)
{
  return ((int16_t)angle + 128) * (MAX_TICKS - MIN_TICKS) / 256 + MIN_TICKS;
}

int8_t servo1_angle;
int8_t servo2_angle;

void servo_init()
{
  // WGM0 0b011 (fast pwm)
  // COM0A 0b11 (set OCR0A on compare match, clear on BOTTOM)
  // COM0B 0b11 (set OCR0B on compare match, clear on BOTTOM)
  // B is temporarily disabled grrrrr
  //TCCR0A = _BV(WGM01) | _BV(WGM00) | _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1)
  //    | _BV(COM0B0); TODO temporary
  TCCR0A = _BV(WGM01) | _BV(WGM00) | _BV(COM0A1) | _BV(COM0A0);
  TCCR0B = 0;

  // enable overflow interrupt
  TIMSK0 = _BV(TOIE0);

  // set output pins
  //DDRD |= _BV(PD5) | _BV(PD6); TODO temporary
  DDRD |= _BV(PD6);

  set_servo1(0);
  set_servo2(0);
}

void servo_pulse()
{
  // set limits, turn on outputs, and start timer

  OCR0A = 255 - angle_to_ticks(servo1_angle);
  OCR0B = 255 - angle_to_ticks(servo2_angle);
  TCNT0 = 0;

  // CS0 0b011 (64 prescaler)
  TCCR0B = _BV(CS01) | _BV(CS00);
}

void set_servo1(int8_t angle)
/* angle must be between -128 and 127 */
{
  servo1_angle = angle;
}

void set_servo2(int8_t angle)
/* angle must be between -128 and 127 */
{
  servo2_angle = angle;
}

int8_t get_servo1()
{
  return servo1_angle;
}

int8_t get_servo2()
{
  return servo2_angle;
}

ISR(TIMER0_OVF_vect)
{
  // stop timer
  TCCR0B = 0;
}
