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
 * @file motors.cpp
 * @brief Contains code to control the motors.
 *
 * @defgroup motors Motors
 * @brief Functions for using the motors
 *
 * @author Colony Project, CMU Robotics Club
 * @author Ben Wasserman
 * @author Tom Mullins
 *
 * @{
 */

#include <ros/ros.h>
#include <cstdlib>
#include "motors.h"

using namespace std;

/**
 * @brief Motor constructor
 *
 * Opens device file and reads initial PWM value
 *
 * @param filename The name of the PWM device file
 */
Motor::Motor(int pwm_gpt, int in1_gpio, int in2_gpio)
{
    char buf[60];

    // open device files
    sprintf(buf, "/dev/pwm%d", pwm_gpt);
    fpwm.open(buf, ios::out);
    sprintf(buf, "/sys/class/gpio/gpio%d/value", in1_gpio);
    fin1.open(buf, ios::out);
    sprintf(buf, "/sys/class/gpio/gpio%d/value", in2_gpio);
    fin2.open(buf, ios::out);

    /* I set speed to 0 here rather than reading in what the current speed is
     * because the pwm driver does not support seeking in the device files.
     * Doing both reading and writing causes fpwm to attempt seeking, which
     * fails, causing fpwm to refuse to do any more io. So, we can only write.
     */
    speed = 0;

    // ensure we are in a consistent state by writing to hardware
    set_speed(speed);
}

/**
 * @brief Motor destructor
 *
 * Sets the motor's speed to zero in preparation for the node exiting
 */
Motor::~Motor()
{
    set_speed(0);
}

/**
 * @brief Returns current motor speed
 *
 * Note that this doesn't read from the hardware
 */
int Motor::get_speed()
{
    return speed;
}

/**
 * @brief Sets motor speed
 *
 * This will set the member this->speed and write the new speed to the hardware
 *
 * @param new_speed The speed to set, between +/- MAXSPEED inclusive
 */
void Motor::set_speed(int new_speed)
{
    int pwm, in1, in2;

    speed = new_speed;

    // convert to hardware units
    if (speed == 0)
    {
        /// @todo should this be off (00) or short brake (11)?
        pwm = 0;
        in1 = 0;
        in2 = 0;
    }
    else if (speed > 0)
    {
        // CW
        pwm = speed;
        in1 = 1;
        in2 = 0;
    }
    else
    {
        // CCW
        pwm = -speed;
        in1 = 0;
        in2 = 1;
    }

    // write to hardware
    fpwm << pwm << flush;
    fin1 << in1 << flush;
    fin2 << in2 << flush;
}

/// @todo change these to the correct motor locations / directions
// Motor state variables
static Motor motor_fl( 8, 70, 71);
static Motor motor_fr( 9, 72, 73);
static Motor motor_bl(11, 74, 75);
static Motor motor_br(10, 76, 77);

/**
 * @brief Sets motor speed
 *
 * Sets the motor speeds based on subscription to the set_motors topic.
 *
 * @param msg The message from the set_motors topic, containing speeds and
 *  motor configuration settings.
 */
void motors_set(const ::messages::set_motors::ConstPtr& msg)
{
    if(msg->fl_set)
    {
        motor_fl.set_speed(msg->fl_speed);
    }
    if(msg->fr_set)
    {
        motor_fr.set_speed(msg->fr_speed);
    }
    if(msg->bl_set)
    {
        motor_bl.set_speed(msg->bl_speed);
    }
    if(msg->br_set)
    {
        motor_br.set_speed(msg->br_speed);
    }

    ROS_DEBUG("Motor speeds set");
}

/**
 * @brief Outputs current motor speeds
 *
 * Serves the service query_motors by responding to service requests with the
 * speeds of the motors.
 * @param req The request. The only field is the units requested.
 * @param res The response. The fields will be filled with values.
 */
bool motors_query(::messages::query_motors::Request &req,
                  ::messages::query_motors::Response &res)
{
    res.fl_speed = motor_fl.get_speed();
    res.fr_speed = motor_fr.get_speed();
    res.bl_speed = motor_bl.get_speed();
    res.br_speed = motor_br.get_speed();

    ROS_DEBUG("Motor speeds queried");

    return true;
}

/**
 * @brief Motors driver. This is a ROS node that controls motor speeds.
 *
 * This is the main function for the motors node. It is run when the node
 * starts and initializes the motors. It then subscribes to the
 * set_motors, and set_motor_speeds topics, and advertises the
 * query_motors service.
 * 
 * @param argc The number of command line arguments (should be 1)
 * @param argv The array of command line arguments
 */
int main(int argc, char **argv)
{
    /* Initialize in ROS the motors driver node */
    ros::init(argc, argv, "motors_driver");

    /* Advertise that this serves the query_motors service */
    ros::NodeHandle node;
    ros::ServiceServer service = node.advertiseService("query_motors",
                                                       motors_query);

    /* Subscribe to the set_motors topic */
    ros::Subscriber sub0 = node.subscribe("set_motors", QUEUE_SIZE, motors_set);

    ROS_INFO("Motors node ready.");
    ros::spin();

    return 0;
}

/** @} */
