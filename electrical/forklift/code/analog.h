#ifndef _ANALOG_H_
#define _ANALOG_H_

#define LWHITE     0
#define LGREY      1
#define LBLACK     2
#define CENTER     3
#define NOLINE   -42
#define LINELOST  -1

#define NOBARCODE     -2
#define INTERSECTION -25
#define FULL_LINE    -26

#define ADC_HEIGHT 0
#define ADC_LINE   1

void analog_init(void);
int analog_read(int which);

void line_update(char* values);
int line_locate(char* values);
int line_read(int which);
int line_read_pos(void);
void line_set_threshold_high(uint8_t threshhold);
void line_set_threshold_low(uint8_t threshhold);

#endif
