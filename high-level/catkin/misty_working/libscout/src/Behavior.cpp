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

/**
 * @file Behavior.cpp
 * @brief Contains basic functions for the structure of all behaviors.
 * 
 * Contains function implementations needed for all behavior.
 *
 * @author Colony Project, CMU Robotics Club
 * @author Priyanka Deo
 * @author Alex Zirbel
 * 
 * @defgroup behavior Behavior
 * @brief Functions which are accessible by all behaviors.
 *
 * @{
 **/

#include "Behavior.h"

using namespace std;

bool Behavior::keep_running;

/**
 * Constructs a behavior and sets up all control classes.
 *
 * This constructor is used by all behaviors which inherit the Behavior
 * class, so that it is easy to create a behavior with access to ROS
 * functionality through the control classes.
 *
 * @param scoutname If nonempty, specifies which scout in the simulator
 *                  will be controlled by this behavior.
 */
Behavior::Behavior(string scoutname, string my_name, Sensors * sensors)
{
    name = my_name;
    keep_running = true;
    sensors->init(&motors, &buttons, &sonar, &encoders, &linesensor,
                        &wl_sender, &wl_receiver, &paintboard, &metal_detector);
    wl_receiver->register_callback(std::bind(&Behavior::default_callback,
        this, std::placeholders::_1));

    loop_rate = new ros::Rate(10);
}

/**
 * Cleans up after a behavior is killed.
 */
Behavior::~Behavior()
{
  motors->set_sides(0, 0, MOTOR_ABSOLUTE);
  spinOnce();
  delete wl_receiver;
  delete loop_rate;
}

/**
 * Empty callback used as default callback for wireless receiver.
 */
void Behavior::default_callback(vector<uint8_t> data)
{
  return;
}

/**
 * Allows the behavior to check ros::ok(), without being aware of ROS.
 */
bool Behavior::ok()
{
    return keep_running;
}

/**
 * Allows the behavior to call ros::spin(), without being aware of ROS.
 */
void Behavior::spin()
{
    ros::spin();
    return;
}

/**
 * Allows the behavior to call ros::spinOnce(), without being aware of ROS.
 */
void Behavior::spinOnce()
{
    ros::spinOnce();
    return;
}

/**
 * Calls ros::spinOnce() in a loop until the duration is up.
 */
void Behavior::wait(float duration)
{
    ros::Rate r(WAIT_HZ);
    int ticks = int(duration * WAIT_HZ);
    for (int i = 0; i < ticks; i++)
    {
        spinOnce();
        r.sleep();
    }
}

/** @} */
