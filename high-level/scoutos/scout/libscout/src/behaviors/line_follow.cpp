/**
 * Copyright (c) 2011 Colony Project
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

#include "line_follow.h"

using namespace std;

static int motor_l;
static int motor_r;

Duration init_turn_time(0.8);

void line_follow::follow_line()
{
    vector<uint32_t> readings;
    do
    {
        double line_loc = linesensor->readline();

        if (line_loc == 0.0)
        {
            motors->set_sides(-60, -60, MOTOR_ABSOLUTE);
        }
        else
        {
            motor_l = min(max((int) (-MOTOR_BASE + KP * line_loc), -128), 127);
            motor_r = min(max((int) (-MOTOR_BASE - KP * line_loc), -128), 127);

            motors->set_sides(motor_l, motor_r, MOTOR_ABSOLUTE);
        }
        readings = linesensor->query();
    }
    while(!linesensor->fullline(readings) &&
          !linesensor->destination(readings) &&
          ok());
    halt();
}

void line_follow::turn_straight()
{
  vector<uint32_t> readings;
  do
  {
    motors->set_sides(-MOTOR_BASE, -MOTOR_BASE, MOTOR_ABSOLUTE);
    readings = linesensor->query();
  }
  while(!linesensor->fullline(readings) &&
        !linesensor->destination(readings) &&
        ok());
}

void line_follow::turn_left()
{
  bool first = true;
  float line_loc;
  do
  {
    motor_l = -MOTOR_BASE;
    motor_r = 0;

    motors->set_sides(motor_l, motor_r, MOTOR_ABSOLUTE);

    if(first)
    {
        init_turn_time.sleep();
        first = false;
    }

    line_loc = linesensor->readline();
  }
  while(!(-1 < line_loc && line_loc < 1) && ok());
}

void line_follow::halt()
{
    motors->set_sides(0, 0, MOTOR_ABSOLUTE);
}

void line_follow::spot_turn()
{
  bool first = true;
  float initial_loc = linesensor->readline();  
  float line_loc = initial_loc;

  do
  {
    motor_l = MOTOR_BASE;
    motor_r = -MOTOR_BASE;

    motors->set_sides(motor_l, motor_r, MOTOR_ABSOLUTE);

    if(first)
    {
        init_turn_time.sleep();
        do 
        {
            line_loc = linesensor->readline();
        }
        while ((abs(line_loc - initial_loc) < 0.0001 ||
                (line_loc < -2 || line_loc > 2)) && ok());
        first = false;
    }
    line_loc = linesensor->readline();
  } 
  while(!(-1 < line_loc && line_loc < 1) && ok());
    //keep repeating until line_loc gets a fresh reading
}

void line_follow::turn_right()
{
  bool first = true;
  float line_loc;
  do
  {
    motor_l = 0;
    motor_r = -MOTOR_BASE;

    motors->set_sides(motor_l, motor_r, MOTOR_ABSOLUTE);

    if(first)
    {
        init_turn_time.sleep();
        first = false;
    }

    line_loc = linesensor->readline();
  }
  while(!(-1 < line_loc && line_loc < 1) && ok());
}

void line_follow::u_turn()
{
  turn_right();
  follow_line();
  turn_right();
}

void line_follow::run()
{
  while(ok())
  {
    follow_line();
  }
}
