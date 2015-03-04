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
 * @file ButtonControl.h
 * @brief Contains buttons declarations and functions
 * 
 * Contains functions and definitions for the use of
 * buttons
 *
 * @author Colony Project, CMU Robotics Club
 * @author Priyanka Deo
 * @author Leon Zhang
 **/

#include "messages/button_event.h"
#include "constants.h"

#ifndef _LIBBUTTONS_H_
#define _LIBBUTTONS_H_

class ButtonControl
{
    public:
        /** Set up buttons node and prepare to communicate over ROS */
        ButtonControl(const ros::NodeHandle& libscout_node,
                      std::string scoutname);

        void event_callback(const messages::button_event::ConstPtr& msg);

        bool button1_is_pressed();
        bool button2_is_pressed();

    private:
        bool button1_value;
        bool button2_value;

        ros::NodeHandle node;

        /** ROS subscriber declaration */
        ros::Subscriber button_event_sub;
};

#endif
