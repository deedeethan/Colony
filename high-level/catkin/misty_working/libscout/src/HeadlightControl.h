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
 * @file HeadlightControl.h
 * @brief Contains headlights declarations and functions
 * 
 * Contains functions and definitions for the use of
 * headlights
 *
 * @author Colony Project, CMU Robotics Club
 * @author Ben Wasserman
 * @author Alex Zirbel
 */

#ifndef _HEADLIGHT_CONTROL_H_
#define _HEADLIGHT_CONTROL_H_

#include <ros/ros.h>
#include <messages/set_headlights.h>

#include "constants.h"

/* Defines */
#define WHITE		0xFFFFFF
#define OFF			0x000000
#define RED 		0xFF0000
#define GREEN 	    0x00FF00
#define BLUE		0x0000FF
#define YELLOW	    0xFFFF00
#define CYAN		0x00FFFF
#define PINK		0xFF00FF

#define HL_RIGHT	0x1
#define HL_LEFT		0x2
#define HL_BOTH		0x3

#define NO_SET	-1

#define REDSHIFT 		16
#define GREENSHIFT 	    8
#define BLUESHIFT		0

class HeadlightControl
{
    public:
        HeadlightControl(const ros::NodeHandle& libscout_node,
                         std::string scoutname);

        void set(int color, int which);
        void set_rgb(int red, int green, int blue, int which);

    private:
        ros::NodeHandle node;

        ros::Publisher set_headlights_pub;
};

#endif

