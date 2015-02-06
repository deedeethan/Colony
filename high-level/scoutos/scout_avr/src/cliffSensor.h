#ifndef _CLIFFSENSOR_H_
#define _CLIFFSENSOR_H_

#include <avr/io.h>
#include <avr/interrupt.h>

void cliffSensor_init(void);
bool read_cliffSensor_front(void);
bool read_cliffSensor_left(void);
bool read_cliffSensor_right(void);

/* The returned result is in the form of left_front_right; 
 * For example, if both left and front sensors detect a cliff, 
 * the returned value is 0b110 */
unsigned char read_cliffSensor_all(void);

#endif
