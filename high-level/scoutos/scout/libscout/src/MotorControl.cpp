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
 * @file MotorControl.cpp
 * @brief Contains motor declarations and functions
 * 
 * @defgroup motorcontrol MotorControl
 * @brief Functions which a behavior can use to control the motors.
 * @ingroup behavior
 *
 * @author Colony Project, CMU Robotics Club
 * @author Ben Wasserman
 * @author Alex Zirbel
 *
 * @{
 **/

#include "MotorControl.h"

using namespace std;

/**
 * @brief Initialize the motors module of libscout.
 *
 * Initialize the libscout node as a publisher of set_motors and a client of
 * query_motors.
 */
MotorControl::MotorControl(const ros::NodeHandle& libscout_node,
                           string scoutname)
    : node(libscout_node)
{
    set_motors_pub =
        node.advertise< ::messages::set_motors>(scoutname + "/set_motors",
                                           QUEUE_SIZE, true);
    query_motors_client =
        node.serviceClient< ::messages::query_motors>(scoutname + "/query_motors");
}

MotorControl::~MotorControl()
{
    set_sides(0, 0, MOTOR_ABSOLUTE);
}

/**
 * @brief Sets all the speeds according to the specificiation of which motors
 *        to set.
 *
 * @param which A bitmask of which motors need to be set.
 * @param speed The value to set the motors to.
 * @param units The units the speed is expressed in.
 * @return Function status
 */
void MotorControl::set(int which, float speed, char units)
{
    float speed_fl, speed_fr, speed_bl, speed_br;
    speed_fl = speed_fr = speed_bl = speed_br = speed;

    if(which & MOTOR_FL_REV)
    {
        speed_fl = -speed;
    }
    if(which & MOTOR_FR_REV)
    {
        speed_fr = -speed;
    }
    if(which & MOTOR_BL_REV)
    {
        speed_bl = -speed;
    }
    if(which & MOTOR_BR_REV)
    {
        speed_br = -speed;
    }

    set_each(which, speed_fl, speed_fr, speed_bl, speed_br, units);
}

/**
 * @brief Sets the left and right sides of scout to different speeds.
 *
 * @param speed_l The speed of both left motors.
 * @param speed_r The speed of both right motors.
 * @param units The units to set to.
 * @return Function status
 */
void MotorControl::set_sides(float speed_l, float speed_r, char units)
{
    set_each(MOTOR_ALL, speed_l, speed_r, speed_l, speed_r, units);
}

/**
 * @brief Set motor speeds
 * Sets the speeds of the motors as a percentage of top speed. Can selectively
 * select which motors to set, and which to keep at previous speed.
 *
 * @param which A bitmask of which motors should be set.
 * @param speed The speed the motors should be set to.
 * @param units Optional units for the speeds.
 * @return Function status
 */
void MotorControl::set_each(int which,
                            float speed_fl, float speed_fr,
                            float speed_bl, float speed_br,
                            char units)
{
    check_which_ok(which);

    ::messages::set_motors msg;


    /* Tell the motors node which motors need to be updated */
    msg.fl_set = msg.fr_set = msg.bl_set = msg.br_set = false;
    if(which & (MOTOR_FL | MOTOR_FL_REV))
    {
        msg.fl_set = true;
        motor_fl_speed = rel_to_abs(speed_fl, units);
    }
    if(which & (MOTOR_FR | MOTOR_FR_REV))
    {
        msg.fr_set = true;
        motor_fr_speed = rel_to_abs(speed_fr, units);
    }
    if(which & (MOTOR_BL | MOTOR_BL_REV))
    {
        msg.bl_set = true;
        motor_bl_speed = rel_to_abs(speed_bl, units);
    }
    if(which & (MOTOR_BR | MOTOR_BR_REV))
    {
        msg.br_set = true;
        motor_br_speed = rel_to_abs(speed_br, units);
    }

    /* Set all the speeds (the booleans in the set_motors message determine
     * which ones will be used) */
    msg.fl_speed = trim_speed("FL", (int) motor_fl_speed);
    msg.fr_speed = trim_speed("FR", (int) motor_fr_speed);
    msg.bl_speed = trim_speed("BL", (int) motor_bl_speed);
    msg.br_speed = trim_speed("BR", (int) motor_br_speed);

    /* Publishes message to set_motors topic */
    set_motors_pub.publish(msg);
    ros::spinOnce();
}

/**
 * Double-checks the which variable to make sure the front and back
 * bits are not both set for a single motor.
 *
 * @param which The which motor specfication to check.
 */
void MotorControl::check_which_ok(int which)
{
    ROS_ERROR_COND( ((which & MOTOR_FL) && (which & MOTOR_FL_REV)),
        "FL Motor set in both directions!");
    ROS_ERROR_COND( ((which & MOTOR_FR) && (which & MOTOR_FR_REV)),
        "FR Motor set in both directions!");
    ROS_ERROR_COND( ((which & MOTOR_BL) && (which & MOTOR_BL_REV)),
        "BL Motor set in both directions!");
    ROS_ERROR_COND( ((which & MOTOR_BR) && (which & MOTOR_BR_REV)),
        "BR Motor set in both directions!");
}

/**
 * Trims an absolute speed to stay within motor maximum range.
 * Uses ROS_WARN to warn the user if a speed has been trimmed.
 *
 * @param which_str A description of which motor, to be used in the warning
 *                  message (for example, "FL")
 * @param speed The requested speed to be set to
 * @return The requested speed, trimmed to stay within limits.
 */
int MotorControl::trim_speed(string which_str, int speed)
{
    int trimmed_speed = speed;
    if (speed > MAXSPEED_ABS)
    {
        trimmed_speed = (int) MAXSPEED_ABS;
        ROS_WARN("Motor (%s) speed trimmed to %d.", which_str.c_str(),
            (int) MAXSPEED_ABS);
    }
    else if (speed < -MAXSPEED_ABS)
    {
        trimmed_speed = (int) -MAXSPEED_ABS;
        ROS_WARN("Motor (%s) speed trimmed to %d.", which_str.c_str(),
            (int) -MAXSPEED_ABS);
    }

    return trimmed_speed;
}

/**
 * @brief Query the current speeds of the motors
 *
 * Sends a request to the query_motors service which will reply with the
 *  current speed of each motor.
 *
 * @todo Change so we can get multiple motor speeds with one call
 *
 * @param which A bitmask that will specify which motor speed should be
 *  returned
 * @return The speed of the selected motor
 */
float MotorControl::query(int which)
{
    ::messages::query_motors srv;
    if(query_motors_client.call(srv))
    {
        switch(which)
        {
            case MOTOR_FL:
                return srv.response.fl_speed;
            case MOTOR_FR:
                return srv.response.fr_speed;
            case MOTOR_BL:
                return srv.response.bl_speed;
            case MOTOR_BR:
                return srv.response.br_speed;
            default:
                /// @todo: I hate this. Let's fix it soon.
                ROS_FATAL("Bad WHICH in motors_query.");
        }
    }
    else
    {
        ROS_FATAL("Failed to call service query_motors");
    }

    return 0;
}

/**
 * @brief Converts set speeds (of various units) to absolute speeds.
 *
 * @param speed The speed expressed in the desired units
 * @param units The units the desired speed is measured in
 * @return The absolute speed of the motor
 **/
float MotorControl::rel_to_abs(float rel_speed, int units)
{
    switch(units)
    {
      case MOTOR_ABSOLUTE:/* Speed given as absolute */
        return rel_speed;
      case MOTOR_PERCENT:/* Convert from percentage */
        return rel_speed * MAXSPEED_ABS / 100.0;
      case MOTOR_MMS:/* Convert from mm/s */
        return rel_speed * MAXSPEED_ABS / (1000.0 * MAXSPEED_MPS);
      case MOTOR_CMS:/* Convert from cm/s */
        return rel_speed * MAXSPEED_ABS / (100.0 * MAXSPEED_MPS);
      default: /* The units aren't recognized */
        ROS_WARN("MotorControl::rel_to_abs used with default units.");
        return rel_speed;
    }
}

/**
 * @brief Convert absolute speeds to speeds of various units.
 *
 * @param speed The speed expressed in absolute units.
 * @param units The units the desired speed is measured in.
 * @return The relative speed of the motor in desired units.
 **/
float MotorControl::abs_to_rel(float abs_speed, int units)
{
    switch(units)
    {
      case MOTOR_ABSOLUTE:/* Speed given as absolute */
        return abs_speed;
      case MOTOR_PERCENT:/* Convert from percentage */
        return abs_speed * 100.0 / MAXSPEED_ABS;
      case MOTOR_MMS:/* Convert from mm/s */
        return abs_speed * MAXSPEED_MPS * 1000.0 / MAXSPEED_ABS;
      case MOTOR_CMS:/* Convert from cm/s */
        return abs_speed * MAXSPEED_MPS * 100.0 / MAXSPEED_ABS;
      default: /* The units aren't recognized */
        ROS_WARN("MotorControl::rel_to_abs used with default units.");
        return abs_speed;
    }
}

/** @} */
