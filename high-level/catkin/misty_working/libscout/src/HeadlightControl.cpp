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
 * @file HeadlightControl.cpp
 * @brief Contains headlights declarations and functions
 * 
 * @defgroup headlightcontrol HeadlightControl
 * @brief Functions which a behavior can use to control the headlights.
 * @ingroup behavior
 *
 * @author Colony Project, CMU Robotics Club
 * @author Ben Wasserman
 * @author Alex Zirbel
 */

#include "HeadlightControl.h"

using namespace std;

/** ROS publisher and client declaration */

/**
 * @brief Initialize the headlights module of libscout.
 *
 * Initialize the libscout node as a publisher of set_headlights.
 */
HeadlightControl::HeadlightControl(const ros::NodeHandle& libscout_node,
                                   string scoutname)
    : node(libscout_node)
{
    set_headlights_pub = node.advertise< ::messages::set_headlights>
        (scoutname + "/set_headlights", QUEUE_SIZE, true);
}

/**
 * @brief Set headlight colors as a single RGB value
 *
 * Sets the colors of the headlights as a web color value. Can selectively
 * select which motors to set, and which to remain at previous color.
 * @param color The color the headlights should be set to expressed as 0xRRGGBB
 * @param which A bitmask of which headlights should be set
 */
void HeadlightControl::set(int color, int which)
{
    int red, green, blue;

    red = (color & RED) >> REDSHIFT;
    green = (color & GREEN) >> GREENSHIFT;
    blue = (color & BLUE) >> BLUESHIFT;

    set_rgb(red, green, blue, which);
}

/**
 * @brief Set headlight colors as separate RGB values
 *
 * Sets the colors of the headlights as a web color value. Can selectively
 * select which motors to set, and which to remain at previous color.
 * @param red The red value the headlights should be set to
 * @param green The green value the headlights should be set to
 * @param blue The blue value the headlights should be set to
 * @param which A bitmask of which headlights should be set
 */
void HeadlightControl::set_rgb(int red, int green, int blue, int which)
{
    ::messages::set_headlights msg;

    if(which & HL_LEFT)
    {
        msg.left_red = red;
        msg.left_green = green;
        msg.left_blue = blue;
    }
    else
    {
        msg.left_red = NO_SET;
        msg.left_green = NO_SET;
        msg.left_blue = NO_SET;
    }
    if(which & HL_RIGHT)
    {
        msg.right_red = red;
        msg.right_green = green;
        msg.right_blue = blue;
    }
    else
    {
        msg.right_red = NO_SET;
        msg.right_green = NO_SET;
        msg.right_blue = NO_SET;
    }

    /* Publishes message to set_motors topic */
    set_headlights_pub.publish(msg);
    ros::spinOnce();
}

/** @} */
