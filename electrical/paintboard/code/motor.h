#ifndef _MOTOR_H_
#define _MOTOR_H_

void motor_init(void);
void set_motor1(int8_t speed);
void set_motor2(int8_t speed);
int8_t get_motor1();
int8_t get_motor2();

#endif
