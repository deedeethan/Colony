#include "twi.h"
#include "motor.h"
#include "servo.h"
#include "sol.h"
#include "geiger.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define TRACKING_ID 0x43
#define SERIAL_NUMBER 0x12

// indicies for paintboard internal data
#define PAINT_TRACKING_ID       0 // ro
#define PAINT_SERIAL_NUMBER     1 // ro
#define PAINT_MOTOR_A           2 // r/w
#define PAINT_MOTOR_B           3 // r/w
#define PAINT_SERVO_A           4 // r/w
#define PAINT_SERVO_B           5 // r/w
#define PAINT_12V_1             6 // r/w
#define PAINT_12V_2             7 // r/w
#define PAINT_12V_3             8 // r/w
#define PAINT_12V_4             9 // r/w
#define PAINT_INPUT_1           10 //ro
#define PAINT_INPUT_2           11 //ro
#define PAINT_INPUT_3           12 //ro


#define PAINT_DATA_LEN         13

#define METAL_DETECT           PD5

uint8_t internal_index = 0;
uint8_t internal_data[PAINT_DATA_LEN] = {
  TRACKING_ID,
  SERIAL_NUMBER,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0
};

void slave_rx(uint8_t* data, int len)
{
  uint8_t value;
  if (len > 0 && data[0] < PAINT_DATA_LEN)
  {
    internal_index = data[0];
    if (len > 1 && internal_index > 1)
    {
      value = data[1];
      internal_data[internal_index] = value;
      switch (internal_index) {
        case PAINT_MOTOR_A:
          set_motor1(value);
          break;
        case PAINT_MOTOR_B:
          set_motor2(value);
          break;
        case PAINT_SERVO_A:
          set_servo1(value);
          break;
        case PAINT_SERVO_B:
          set_servo2(value);
          break;
        case PAINT_12V_1:
          set_sol1(value);
          break;
        case PAINT_12V_2:
          set_sol2(value);
          break;
        case PAINT_12V_3:
          set_sol3(value);
          break;
        case PAINT_12V_4:
          set_sol4(value);
          break;
      }
    }
  }
}

void slave_tx(void)
{
  twi_transmit(&internal_data[internal_index], 1);
  internal_index++;
  if (internal_index >= PAINT_DATA_LEN)
    internal_index = 0;
}

int main()
{
  sei();
  twi_attachSlaveRxEvent(slave_rx);
  twi_attachSlaveTxEvent(slave_tx);
  twi_setAddress(TRACKING_ID);
  twi_init();
  geiger_init();
  motor_init();
  servo_init();
  while (1)
  {
    geiger_tick();
    internal_data[PAINT_INPUT_2] = geiger_rate();
    internal_data[PAINT_INPUT_1] = !(!(_BV(METAL_DETECT) & PIND));
    if (internal_data[PAINT_INPUT_1]) {
      set_servo1(-128);
      set_motor1(0);
      set_motor2(0);
    } else {
      set_servo1(127);
      set_motor1(127);
      set_motor2(127);
    }
    servo_pulse();
    _delay_ms(20);
  }
  return 0;
}
