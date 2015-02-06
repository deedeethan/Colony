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
 * @file PaintBoardControl.h
 * @brief Contains motor declarations and functions
 * 
 * Contains functions and definitions for the use of
 * motors
 *
 * @author Colony Project, CMU Robotics Club
 * @author Priyanka Deo
 **/

#ifndef _PAINT_BOARD_CONTROL_H_
#define _PAINT_BOARD_CONTROL_H_

#include <ros/ros.h>
#include "constants.h"
#include <messages/set_paintboard.h>
#include <messages/query_paintboard.h>

#define STOP_PAINT 0
#define DISPENSE_PAINT 1

class PaintBoardControl
{
    public:
        /** Set up the paintboard node and prepare to communicate over ROS */
        PaintBoardControl(const ros::NodeHandle& libscout_node,
                     std::string scoutname);

        ~PaintBoardControl();

        /** Turns the paintboard on or off. */
        void set(int paintboard_setting);

        /** Checks whether the paintboard is currently dispensing paint */
        int query();
    
    private:
        /** ROS publisher and client declaration */
        ros::Publisher set_paintboard_pub;
        ros::ServiceClient query_paintboard_client;

        ros::NodeHandle node;
};

#endif

