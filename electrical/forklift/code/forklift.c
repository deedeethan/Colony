/*
 * ARM -> us
 * -height setpoint
 *
 * us -> ARM
 * -tracking ID (same for all forklifts)
 * -serial number (unique to each forklift)
 * -current height
 * -current height setpoint
 * -payload weight
 *
 */
 
#include "twi.h"
#include "analog.h"
#include "motor.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define TRACKING_ID 0x41
#define SERIAL_NUMBER 0x12

// indicies for forklift internal data
#define FORKLIFT_TRACKING_ID       0
#define FORKLIFT_SERIAL_NUMBER     1
#define FORKLIFT_HEIGHT            2
#define FORKLIFT_HEIGHT_SETPOINT   3 // r/w
#define FORKLIFT_LINE_POS          4
#define FORKLIFT_LINE_THRESH_HIGH  5 // r/w
#define FORKLIFT_LINE_THRESH_LOW   6 // r/w
#define FORKLIFT_LINE_VALS_START   7
#define FORKLIFT_LINE_VALS_END    12 // non-inclusive

#define FORKLIFT_DATA_LEN         12

uint8_t internal_index = 0;
uint8_t internal_data[] = {
  TRACKING_ID,
  SERIAL_NUMBER,
  0,
  100, // default height setpoint
  0,
  0,
  150, // default line threshold
  0,
  0,
  0,
  0,
  0
};

int error;
int i_term;

void init_int0(void)
{
  TCCR0B = (1<<CS02)|(1<<CS00); //Timer clock = system clock / 1024
  TIFR0 = 1<<TOV0; //Clear TOV0  clear pending interrupts
  TIMSK0 = 1<<TOIE0; //Enable Timer0 Overflow Interrupt
  
}
void slave_rx(uint8_t* data, int len)
{
  if (len > 0 && data[0] < FORKLIFT_DATA_LEN)
  {
    internal_index = data[0];
    if (len > 1)
    {
      if (internal_index == FORKLIFT_HEIGHT_SETPOINT)
      {
          internal_data[internal_index] = data[1];
          i_term = 0;
      }
      else if (internal_index == FORKLIFT_LINE_THRESH_HIGH)
      {
        internal_data[internal_index] = data[1];
        line_set_threshold_high(data[1]);
      }
      else if (internal_index == FORKLIFT_LINE_THRESH_LOW)
      {
        internal_data[internal_index] = data[1];
        line_set_threshold_low(data[1]);
      }
    }
  }
}

void slave_tx(void)
{
  twi_transmit(&internal_data[internal_index], 1);
  internal_index++;
  if (internal_index >= FORKLIFT_DATA_LEN)
    internal_index = 0;
}

int clamp(int min, int max, int val)
{
  if (val > max) return max;
  if (val < min) return min;
  return val;
}

SIGNAL(TIMER0_OVF_vect)
{
  // TODO make this more easily tunable (despite integer limitations)
  int height = (int)internal_data[FORKLIFT_HEIGHT];
  error = (int)internal_data[FORKLIFT_HEIGHT_SETPOINT] - height;
  i_term = i_term + error/4;
  int speed = clamp(-127, 127, i_term/12 + error*4);
  if (height < 15 && speed < 0) speed = 0;
  if (height > 240 && speed > 0) speed = 0;
  set_motor(speed);
}

int main()
{
  int i;
  error = 0;
  i_term = 0;
  init_int0();
  sei();
  twi_attachSlaveRxEvent(slave_rx);
  twi_attachSlaveTxEvent(slave_tx);
  twi_setAddress(TRACKING_ID);
  twi_init();
  analog_init();
  motor_init();
  while (1)
  {
    for (i = 0; i < 5; i++)
      internal_data[i + FORKLIFT_LINE_VALS_START] = line_read(i) >> 2;
    internal_data[FORKLIFT_LINE_POS] = line_read_pos();
    internal_data[FORKLIFT_HEIGHT] = 255 - (analog_read(ADC_HEIGHT) >> 2);
  }
  return 0;
}
