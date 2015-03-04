/**
 * Copyright (c) 2012 Colony Project
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
 * @file WirelessReceiver.cpp
 * @brief Contains wireless declarations and functions
 * 
 * @defgroup wirelessreceiver WirelessReceiver
 * @brief Contains functions and definitions for the use of XBee wireless
 * @ingroup behavior
 *
 * @author Colony Project, CMU Robotics Club
 * @author Tom Mullins
 *
 * @{
 **/

#include "WirelessReceiver.h"

using namespace std;

/**
 * @brief Initializes wireless receiver
 * 
 * @param libscout_node Node handle of the ROS node used by libscout.
 **/
WirelessReceiver::WirelessReceiver(const ros::NodeHandle& libscout_node)
    : node(libscout_node), callback(bind(&WirelessReceiver::dummy, this, _1))
{
    sub = node.subscribe("/wireless/receive", QUEUE_SIZE,
          &WirelessReceiver::receive, this);
}

/**
 * @brief Registers a function to be called upon receipt of a packet.
 *
 * @param callback A function pointer to a function which is called whenever a
 * wireless packet is received. It should be of form:
 *     void callback(const WirelessPacketConstPtr& packet);
 **/
void WirelessReceiver::register_callback(WirelessCallback new_callback)
{
    callback = new_callback;
}

/**
 * @brief The callback function to pass packets along to the user
 * 
 * @todo Add some kind of filtering functionality
 **/
void WirelessReceiver::receive(const ::messages::WirelessPacketConstPtr& packet)
{
    callback(packet->data);
}

/**
 * @brief The default callback function, to be overwritten by register_callback
 */
void WirelessReceiver::dummy(std::vector<uint8_t> data) {
    ROS_WARN("Warning: Ignored wireless packet.");
}

/** @} */
