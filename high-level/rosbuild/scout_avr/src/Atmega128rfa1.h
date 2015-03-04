#ifndef _ATMEGA128RFA1_H_
#define _ATMEGA128RFA1_H_

#include "ros/node_handle.h"

#define BAUD_RATE 38400

#define RX_BUFFER_SIZE 256

#define MAX_SUBSCRIBERS 3
#define MAX_PUBLISHERS 5
#define INPUT_SIZE 256
#define OUTPUT_SIZE 1024

class Atmega128rfa1
{
public:
  Atmega128rfa1();
  void init();
  int read();
  void write(uint8_t* data, int length);
  void puts(const char* str) {write((uint8_t*) str, strlen(str));}
  unsigned long time();
};

#endif
