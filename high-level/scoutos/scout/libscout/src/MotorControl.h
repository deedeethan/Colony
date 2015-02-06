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
 * @file MotorControl.h
 * @brief Contains motor declarations and functions
 * 
 * Contains functions and definitions for the use of
 * motors
 *
 * @author Colony Project, CMU Robotics Club
 * @author Ben Wasserman
 * @author Alex Zirbel
 **/

#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include <ros/ros.h>
#include <messages/query_motors.h>
#include <messages/set_motors.h>

#include "constants.h"

/* Motor-specific defines */
#define MOTOR_ALL 0xF
#define MOTOR_ALL_REV 0xF0
#define MOTOR_NONE 0x0
#define MOTOR_FL 0x8
#define MOTOR_FR 0x4
#define MOTOR_BL 0x2
#define MOTOR_BR 0x1
#define MOTOR_FL_REV 0x80
#define MOTOR_FR_REV 0x40
#define MOTOR_BL_REV 0x20
#define MOTOR_BR_REV 0x10
#define MOTOR_FRONT MOTOR_FL | MOTOR_FR
#define MOTOR_BACK MOTOR_BR | MOTOR_BR
#define MOTOR_LEFT MOTOR_FL | MOTOR_BL
#define MOTOR_RIGHT MOTOR_FR | MOTOR_BR
#define MOTOR_LEFT_REV MOTOR_FL_REV | MOTOR_BL_REV
#define MOTOR_RIGHT_REV MOTOR_FR_REV | MOTOR_BR_REV
#define MOTOR_LEFT_SPIN MOTOR_LEFT_REV | MOTOR_RIGHT
#define MOTOR_RIGHT_SPIN MOTOR_LEFT | MOTOR_RIGHT_REV

#define MAXSPEED_ABS 127.0f
#define MAXSPEED_MPS 1.0f
#define MOTOR_PERCENT 'p'
#define MOTOR_MMS 'm'
#define MOTOR_CMS 'c'
#define MOTOR_ABSOLUTE 'a'
#define MOTOR_DEFAULT_UNIT MOTOR_PERCENT

class MotorControl
{
    public:
        /** Set up the motor node and prepare to communicate over ROS */
        MotorControl(const ros::NodeHandle& libscout_node,
                     std::string scoutname);

        ~MotorControl();

        /** Uses which to specify what to change, and sets all to same speed */
        void set(int which, float speed, char units=MOTOR_DEFAULT_UNIT);

        /** Sets each side to a different speed */
        void set_sides(float speed_l, float speed_r,
                     char units=MOTOR_DEFAULT_UNIT);

        /** Sets each motor speed individually */
        void set_each(int which,
                    float speed_fl, float speed_fr,
                    float speed_bl, float speed_br,
                    char units=MOTOR_DEFAULT_UNIT);
        
        /** Requests the current motor speeds */
        float query(int which);
    
    private:
        /** Error if which sets a motor to both forward and backward */
        void check_which_ok(int which);
        int trim_speed(std::string which_str, int speed);

        float rel_to_abs(float rel_speed, int units);
        float abs_to_rel(float abs_speed, int units);

        /* Absolute speeds, but allowing fractions. Useful for PID, etc. */
        float motor_fl_speed;
        float motor_fr_speed;
        float motor_bl_speed;
        float motor_br_speed;

        /** ROS publisher and client declaration */
        ros::Publisher set_motors_pub;
        ros::ServiceClient query_motors_client;

        ros::NodeHandle node;
};

#endif

