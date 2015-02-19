#ifndef _SERVO_H_
#define _SERVO_H_

void servo_init();
void servo_pulse(); // should be called ~every 20ms
void set_servo1(int8_t angle);
void set_servo2(int8_t angle);
int8_t get_servo1();
int8_t get_servo2();

#endif
