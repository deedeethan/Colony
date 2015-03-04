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
 **/

/**
 * @file CliffsensorControl.cpp
 * @brief Contains code to control the cliffsensor.
 *
 * @defgroup motorcontrol MotorControl
 * @brief Functions which a behavior can use to control the motors.
 * @ingroup behavior
 *
 * @author Colony Project, CMU Robotics Club
 * @author Priyanka Deo
 * @author Leon Zhang
 *
 * @{
 **/

#include "ros/ros.h"
#include "CliffsensorControl.h"
#include <cstdlib>

/**
 * @brief Initializes the Cliffsensor module in libscout.
 *
 * This is the main function for the cliffsensors node. It is run when the node
 * starts and initializes the cliffsensors. It then subscribes to the
 * cliff_status_changed topics
 **/
CliffsensorControl::CliffsensorControl(const ros::NodeHandle& libscout_node,
                                  std::string scoutname) : node(libscout_node)
{
    /* Subscribe to the cliff_status_changed topic */
    cliff_status_changed_sub = node.subscribe("cliff_status_changed", 
                             QUEUE_SIZE, &CliffsensorControl::changed_cliff_status, this);

    ros::spin();
}

/**
 * @brief Changes cliff sensor status
 *
 * Changes cliff sensor status based on subscription to topic cliff_status_changed
 *
 * @param msg The message from the cliff_status_changed topic, containing 
 * status of all cliff sensors.
 *
 */
void CliffsensorControl::changed_cliff_status(const ::messages::cliff_status_changed::ConstPtr& msg)
{
    front_raw = msg->cliff_status;
    left_raw = msg->cliff_status;
    right_raw = msg->cliff_status;
    return;
}

/**
 * @brief get the current raw value of the front cliffsensor
 * @return the current raw value of the front cliffsensor
 */
int CliffsensorControl::get_front_raw()
{
    return front_raw;
}

/**
 * @brief get the current raw value of the left cliffsensor
 * @return the current raw value of the left cliffsensor
 */
int CliffsensorControl::get_left_raw()
{
    return left_raw;
}

/**
 * @brief get the current raw value of the right cliffsensor
 * @return the current raw value of the right cliffsensor
 */
int CliffsensorControl::get_right_raw()
{
    return right_raw;
}

/**
 * @brief check if a cliff is being detected
 * @return true if there is a cliff, otherwise
 */
bool CliffsensorControl::check_is_cliff()
{
    return (front_raw || left_raw || right_raw);
}

/** @} */
