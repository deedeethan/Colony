/**
 * Copyright (c) 2013 Colony Project
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @file paint_i2c.cpp
 * @brief Contains i2c communication for the paintboard.
 *
 * @author Colony Project, CMU Robotics Club
 * @author Maung Aung (mza)
 * @author Tom Mullins (tmullins)
 *
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include "paint-i2c.h"

#define TRACKING_ID 0x43
#define SERIAL_NUMBER 0x12

// indicies for paintboard internal data
#define PAINT_TRACKING_ID      0 // ro
#define PAINT_SERIAL_NUMBER    1 // ro
#define PAINT_MOTOR_A          2 // rw
#define PAINT_MOTOR_B          3 // rw
#define PAINT_SERVO_A          4 // rw
#define PAINT_SERVO_B          5 // rw
#define PAINT_12V_1            6 // rw
#define PAINT_12V_2            7 // rw
#define PAINT_12V_3            8 // rw
#define PAINT_12V_4            9 // rw
#define PAINT_INPUT_1         10 // ro
#define PAINT_INPUT_2         11 // ro
#define PAINT_INPUT_3         12 // ro

#define PAINT_DATA_LEN        13

int fd;

/*
 * This function opens communication to I2C.
 */
void i2c_start(void) {
  /* TODO error check */
  fd = open("/dev/i2c-3", O_RDWR);
  if (fd < 0) {
    perror("/dev/i2c-3");
    exit(1);
  }
  if (ioctl(fd, I2C_SLAVE, TRACKING_ID)) {
    perror("ioctl");
    exit(1);
  }
}

/*
 * This function ends communication with I2C.
 */
void i2c_stop(void) {
  if (fd) {
    close(fd);
  }
}

/*
 * This function writes a value to a given destination using I2C.
 */
void i2c_write(int dest, int val) {
  char buf[2];
  buf[0] = (char) dest;
  buf[1] = (char) val;

  if (write(fd, buf, 2) < 2) {
    fprintf(stderr, "Warning: i2c_write failed\n");
  }
}

/*
 * This function reads a value from the given address using I2C
 */
char i2c_read(int src) {
  char buf;

  buf = src;
  if (write(fd, &buf, 1) < 1) {
    fprintf(stderr, "Warning: write in i2c_read failed\n");
  } else if (read(fd, &buf, 1) < 1) {
    fprintf(stderr, "Warning: read in i2c_read failed\n");
  }
  return buf;
}

/*
 * This function takes in an int representing the motor to be modified and an 
 * int representing the value to which to set the motor.
 */
void set_motor(int motor, int val) {
  if (motor == 0) {
    i2c_write(PAINT_MOTOR_A, val);
  } else {
    i2c_write(PAINT_MOTOR_B, val);
  }
}

/*
 * This function takes in an int representing the servo to be modified and an 
 * int representing the value to which to set the servo.
 */
void set_servo(int servo, int val) {
  if (servo == 0) {
    i2c_write(PAINT_SERVO_A, val);
  } else {
    i2c_write(PAINT_SERVO_B, val);
  }
}

/*
 * This function takes in an int representing the solenoid to be modified and
 * an int representing the value to which to set the solenoid.
 */
void set_solenoid(int solenoid, int val) {
  if (solenoid == 0) {
    i2c_write(PAINT_12V_1, val);
  } else if (solenoid == 1) {
    i2c_write(PAINT_12V_2, val);
  } else if (solenoid == 2) {
    i2c_write(PAINT_12V_3, val);
  } else {
    i2c_write(PAINT_12V_4, val);
  }
}

/*
 * This funciton takes an int representing which of the spare inputs to read.
 */
int get_input(int input) {
  switch (input) {
    case 0: return i2c_read(PAINT_INPUT_1);
    case 1: return i2c_read(PAINT_INPUT_2);
    case 2: return i2c_read(PAINT_INPUT_3);
    default: return 0;
  }
}
