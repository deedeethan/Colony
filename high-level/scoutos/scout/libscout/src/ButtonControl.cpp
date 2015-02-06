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
 * @file ButtonControl.cpp
 * @brief Contains buttons declarations and functions
 * 
 * @defgroup buttoncontrol ButtonControl
 * @brief Functions which a behavior can use to control the buttons.
 * @ingroup behavior
 *
 * @author Colony Project, CMU Robotics Club
 * @author Priyanka Deo
 * @author Leon Zhang
 * @author Alex Zirbel
 *
 * @{
 **/

#include "ButtonControl.h"

using namespace std;

/**
 * @brief Initialize the buttons module of libscout.
 *
 * Initialize the libscout node as a subscriber of button_event
 */
ButtonControl::ButtonControl(const ros::NodeHandle& libscout_node,
                             string scoutname)
    : node(libscout_node)
{
    button_event_sub = node.subscribe(scoutname + "/button_event",
                                      QUEUE_SIZE,
                                      &ButtonControl::event_callback,
                                      this);
}

/**
 * @brief Respond to button events
 * Processes message and updates private variables
 */
void ButtonControl::event_callback(const ::messages::button_event::ConstPtr& msg)
{
    button1_value = msg->button1_pressed;
    button2_value = msg->button2_pressed;
}

/**
 * @brief check if button 1 is currently being pressed
 * 
 * @return true if button is pressed, otherwise false
 */
bool ButtonControl::button1_is_pressed()
{
    return button1_value;
}

/**
 * @brief check if button 2 is currently being pressed
 * 
 * @return true if button is pressed, otherwise false
 */
bool ButtonControl::button2_is_pressed()
{ 
    return button2_value;
}

/** @} */
