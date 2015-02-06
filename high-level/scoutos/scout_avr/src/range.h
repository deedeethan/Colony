#ifndef _RANGE_H_
#define _RANGE_H_

#define RANGE_ERR 0xFFFF

#define PIN_SONAR_PWM PIND
#define P_SONAR_PWM1 PD2
#define P_SONAR_PWM0 PD3

#define PORT_SONAR_TX PORTG
#define P_SONAR_TX PG1

// initializes timer 5, also used by bom
void range_init();

// blocks during measurement (up to 37.5ms for each)
// writes values into array of 2 unsigned ints
void range_measure(unsigned int *values);

#endif
