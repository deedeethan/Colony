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
 * @file buttons.cpp
 * @brief Contains code to control the buttons.
 *
 * @defgroup buttons Buttons
 * @brief Functions for using the buttons
 *
 * @author Colony Project, CMU Robotics Club
 * @author Priyanka Deo
 * @author Alex Zirbel
 *
 * @{
 **/

int main (int argc, char **argv) {
    return 0;
}
#include "ros/ros.h"
#include "buttons.h"
#include <cstdlib>
//
///* Button state variables
// */
//static int button1_pressed; /**< Whether or not button 1 is pressed. */
//static int button2_pressed; /**< Whether or not button 2 is pressed. */
//
///**
// * @brief Buttons driver. This is a ROS node that queries button status.
// *
// * This is the main function for the buttons node. It is run
// * when the node starts and initializes the buttons. It then 
// * subscribes to the event_button topics, and advertises the 
// * query_buttons service.
// * 
// * @param argc The number of command line arguments (should be 1)
// * @param argv The array of command line arguments
// */
//int main(int argc, char **argv)
//{
//    /* Initialize in ROS the buttons driver node */
//    ros::init(argc, argv, "buttons_driver");
//
//    /* Advertise that this serves the query_buttons service */
//    ros::NodeHandle n;
//
//    ros::Publisher pub = n.advertise< ::messages::button_event>("button_event",
//                                                                QUEUE_SIZE);
//
//    /* Initialize hardware for buttons */
//    // Hardware init functions here
//    
//    // Publish a button event now and then
//
//    ROS_INFO("Ready to handle buttons. [unimplemented]");
//    ros::spin();
//
//    return 0;
//}
//
///** @} **/
