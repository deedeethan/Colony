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
 * @file power.cpp
 * @brief Contains code to monitor and regulate power systems
 *
 * @defgroup power Power
 * @brief Functions for monitoring and regulating the power systems
 *
 * @author Colony Project, CMU Robotics Club
 * @author Jeff Cooper
 *
 * @{
 **/

#include "ros/ros.h"
#include "power.h"
#include <cstdlib>

/* Power system state variables
 * updated by the AVR, used to respond to
 * power queries and send messages
 *
 */

/** @todo Fix types: static */

/** @todo figure out if we can get things like the current draw from the AVR. */
/** @todo More generally, figure out how we get info from the AVR (avrbridge
 * node?) */

uint32_t voltage; /**< the current voltage */
uint32_t percentage; /**< current percentage of power remaining */
uint32_t draw; /**< the current draw in mW */
/** @todo: figure out if these have to be uint8_t's to play nice with ROS */
bool externalpower; /**< are we on external power? */
bool warning; /**< is the battery reporting a warning state? */
bool critical; /**< is the battery reporting a critical state? */

/**
 * @brief Outputs current power state information
 *
 * Serves the service query_power by responding to service requests with the
 * state of the power system
 * @param req The request. There are no fields
 * @param res The response. The fields will be filled with values.
 */
bool power_query(::messages::query_power::Request &req,
                 ::messages::query_power::Response &res)
{
    res.voltage = voltage;
    res.percentage = percentage;
    res.draw = draw;
    res.externalpower = externalpower;
    res.warning = warning;
    res.critical = critical;

    ROS_DEBUG("Power speeds queried");
    return true;
}


/** @todo: implement a function to send a message about the power state when
 * it's critical. can't do much with this until I have some way of reading the
 * power state from the AVR, I don't believe. */


/**
 * @brief Power driver. This is a ROS node that monitors and regulates the power
 * systems
 *
 * This is the main function for the power node. It is run when the node
 * starts. It advertises the query_power service.
 * 
 * @param argc The number of command line arguments (should be 1)
 * @param argv The array of command line arguments
 **/
int main(int argc, char **argv)
{
    /* Initialize in ROS the motors driver node */
    ros::init(argc, argv, "power_driver");

    /* Advertise that this serves the query_motors service */
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("query_power",
                                                    power_query);

    /* Initialize hardware for motors */
    // Hardware init functions here

    ROS_INFO("Ready to set motors.");
    ros::spin();

    return 0;
}

/** @} **/
