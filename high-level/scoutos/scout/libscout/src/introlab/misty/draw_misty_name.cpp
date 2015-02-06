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

#include "draw_misty_name.h"

/**
 * In here we define the run function which specifies the behavior
 * that the scout will run.
 */
void draw_misty_name::turnRight() {
    Duration one_second(0.954);
    motors->set_sides(26, -26, MOTOR_ABSOLUTE);
    one_second.sleep();
}
void draw_misty_name::run()
{
  /**
   * This loop runs as long as ROS is ok. This is necessary to make
   * sure that our program terminates if something is wrong with ROS.
   * And if not continues to run.
   */
    Duration half_second(0.5);
    Duration one_second(1.0);
    Duration one_point_five(1.5);
    Duration three_second(3.0);
    Duration weird_sleep(1.1);
    while(ok()){
        one_second.sleep();

        // move to a good position
        turnRight();
        turnRight();
        motors->set_sides(100, 100, MOTOR_ABSOLUTE);
        half_second.sleep();
        turnRight();
        turnRight();
        turnRight();



        // first edge of M
        motors->set_sides(100,100, MOTOR_ABSOLUTE);
        three_second.sleep();

        // turn at top left
        motors->set_sides(30,-30, MOTOR_ABSOLUTE);
        one_second.sleep();
        one_second.sleep();

        // second edge of M
        motors->set_sides(100,100, MOTOR_ABSOLUTE);
        one_point_five.sleep();

        // turn at middle
        motors->set_sides(-30, 30, MOTOR_ABSOLUTE);
        one_second.sleep();
        one_second.sleep();

        // third edge of M
        motors->set_sides(100,100, MOTOR_ABSOLUTE);
        one_point_five.sleep();

        // turn at top right
        motors->set_sides(127,-127, MOTOR_ABSOLUTE);
        one_second.sleep();
        one_second.sleep();

        // last edge!!!
        motors->set_sides(100,100, MOTOR_ABSOLUTE);
        three_second.sleep();

        // starting to draw i!!! which should be farely simple!!!
        motors->set_sides(-100,100, MOTOR_ABSOLUTE);
        one_point_five.sleep();

        motors->set_sides(-20,100, MOTOR_ABSOLUTE);
        one_second.sleep();
        one_second.sleep();
        one_second.sleep();

        motors->set_sides(50, 50, MOTOR_ABSOLUTE);
        one_second.sleep();

        motors->set_sides(-50, -50, MOTOR_ABSOLUTE);
        one_second.sleep();


        motors->set_sides(-100, 20, MOTOR_ABSOLUTE);
        one_second.sleep();
        one_second.sleep();
        one_second.sleep();

        // start drawing s
        motors->set_sides(127,-127, MOTOR_ABSOLUTE);
        one_second.sleep();
        weird_sleep.sleep();



        motors->set_sides(-50,127, MOTOR_ABSOLUTE);
        one_second.sleep();
        one_second.sleep();

        motors->set_sides(127, -50, MOTOR_ABSOLUTE);
        one_second.sleep();
        one_second.sleep();

        // come back to s
        motors->set_sides(-127, 50, MOTOR_ABSOLUTE);
        one_second.sleep();
        one_second.sleep();

        motors->set_sides(50,-127, MOTOR_ABSOLUTE);
        one_second.sleep();
        one_second.sleep();

        // start drawing t
        motors->set_sides(100,-100, MOTOR_ABSOLUTE);
        one_second.sleep();

        motors->set_sides(-20,100, MOTOR_ABSOLUTE);
        one_second.sleep();
        one_second.sleep();
        one_second.sleep();

        motors->set_sides(50, 50, MOTOR_ABSOLUTE);
        one_second.sleep();

        // a + at the tip of t

        //left
        motors->set_sides(-50,50, MOTOR_ABSOLUTE);
        one_second.sleep();
        one_second.sleep();
        one_point_five.sleep();

        motors->set_sides(50,50, MOTOR_ABSOLUTE);
        one_second.sleep();

        motors->set_sides(-50,-50, MOTOR_ABSOLUTE);
        one_second.sleep();

        // top
        motors->set_sides(50,-50, MOTOR_ABSOLUTE);
        one_second.sleep();
        one_point_five.sleep();
        one_second.sleep();

        motors->set_sides(50,50, MOTOR_ABSOLUTE);
        one_second.sleep();

        motors->set_sides(-50,-50, MOTOR_ABSOLUTE);
        one_second.sleep();

        // right
        motors->set_sides(50,-50, MOTOR_ABSOLUTE);
        one_point_five.sleep();
        one_second.sleep();
        one_second.sleep();

        motors->set_sides(50,50, MOTOR_ABSOLUTE);
        one_second.sleep();

        motors->set_sides(-50,-50, MOTOR_ABSOLUTE);
        one_second.sleep();

        // down 
        motors->set_sides(-50,50, MOTOR_ABSOLUTE);
        one_second.sleep();
        one_point_five.sleep();
        one_second.sleep();

        motors->set_sides(-50, -50, MOTOR_ABSOLUTE);
        one_second.sleep();


        motors->set_sides(-100, 20, MOTOR_ABSOLUTE);
        one_second.sleep();
        one_second.sleep();

        // start drawing y
        motors->set_sides(-100, 100, MOTOR_ABSOLUTE);
        one_second.sleep();
        one_second.sleep();
        one_second.sleep();
        one_point_five.sleep();

        motors->set_sides(-30, 127, MOTOR_ABSOLUTE);
        one_second.sleep();

        motors->set_sides(50, 50, MOTOR_ABSOLUTE);
        one_second.sleep();

        motors->set_sides(-50,-50, MOTOR_ABSOLUTE);
        one_second.sleep();

        motors->set_sides(-127, 70, MOTOR_ABSOLUTE);
        one_point_five.sleep();
        one_point_five.sleep();

        motors->set_sides(-50,-50, MOTOR_ABSOLUTE);
        one_second.sleep();

        motors->set_sides(110, 110, MOTOR_ABSOLUTE);
        one_point_five.sleep();

        motors->set_sides(100,-40, MOTOR_ABSOLUTE);
        three_second.sleep();
        three_second.sleep();

        motors->set_sides(100,100, MOTOR_ABSOLUTE);
        one_second.sleep();
        one_second.sleep();

        ROS_INFO("hallo!");
        /* WRITE YOUR SCOUT BEHAVIOR HERE */

        /**
         * These two functions cause the world to update its
         * state (spinOnce) and causes us to wait for incoming
         * messages (loop_rate->sleep). These are necessary for
         * ROS to do its job. You can think of this as ROS voodoo magic.
         */

        spinOnce();
        loop_rate->sleep();
        motors->set_sides(0,0,MOTOR_ABSOLUTE);
        break;
      }
}
