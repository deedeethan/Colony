extern "C"
{
#include <avr/io.h>
#include <util/delay.h>
}
#include "stepper.h"

/*  Stepper Motor:
 *  Provides interface to stepper motor.
 *  Can set direction. Outputs pulse to stepper upon call of step func
 *  Also provides variable speed sweep
 */

struct step_t {
  int pos; // position in rotation.
  int dir; // direction. -1 CCW. 1 CW. 0 OFF
  int step_size; // amount to add to position each step
  int ccw;
  int cw;
} step;


void step_init()
{
  /* init pos and dir to 0 */
  step.pos = 0;
  step.dir = 0;

  //set control pins as output
  DDRD |= ((1<<S_STEP) | (1<<S_DIR));
  DDRB |= ((1<<S_MS));
  
  /* this is connected to ENABLE temporarily */
  DDRF |= _BV(S_EN);
  step_disable();

  //initiate to full steps
  step_set_size(STEP_WHOLE);
 
  //initiate the step pin to be low. stepper steps on low to high
  PORTD &= (~(1<<S_STEP));
}

void step_enable()
{
  return; // stepper is temporarily disabled for demo
  PORTF &= ~_BV(S_EN);
}

void step_disable()
{
  PORTF |= _BV(S_EN);
}

void step_set_size(char size)
{
  if (size == STEP_WHOLE)
  {
    PORTB &= ~_BV(S_MS);
    step.step_size = 2;
  }
  else
  {
    PORTB |= _BV(S_MS);
    step.step_size = 1;
  }
}

/* set direction pin */
void step_dir(int dir)
{
  step.dir = dir;
  switch(dir)
  {
    case 1:
      PORTD |= (1<<S_DIR);
      break;
    case -1:
      PORTD &= (~(1<<S_DIR));
      break;
  }
}

void step_do_step()
{
  return; // stepper is temporarily disabled for demo
  if(step.dir==0) return; //do not step if not enabled
  PORTD |= (1<<S_STEP); //step once 
  _delay_us(1); //conform with step timing
  PORTD &= (~(1<<S_STEP)); //bring the step bin back down
  if(step.dir==1) step.pos += step.step_size;
  else step.pos -= step.step_size;
}

void step_flush()
{
  PORTD &= (~(1<<S_STEP)); //bring the step bin back down
}

//ccw must be less than 0 and cw must be greater than 0
void step_sweep_bounds(int ccw, int cw)
{
  step.ccw = ccw;
  step.cw = cw;
}

void step_sweep()
{
  step_do_step();
  if((step.dir == 1) && (step.cw <= step.pos)) step_dir(-1);
  else if((step.dir == -1) && (step.ccw >= step.pos)) step_dir(1);
}

int step_get_pos()
{
  return step.pos;
}
