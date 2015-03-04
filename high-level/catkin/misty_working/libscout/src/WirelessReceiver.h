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
 * @file WirelessReceiver.h
 * @brief Contains wireless declarations and functions
 * 
 * Contains functions and definitions for the use of
 * XBee wireless
 *
 * @author Colony Project, CMU Robotics Club
 * @author Tom Mullins
 **/

#ifndef _WIRELESS_RECEIVER_H_
#define _WIRELESS_RECEIVER_H_

#include <vector>
#include <functional>
#include <ros/ros.h>
#include "messages/WirelessPacket.h"
#include "constants.h"

typedef std::function<void(std::vector<uint8_t>)> WirelessCallback;

class WirelessReceiver
{
    public:
        /** Set up the class and prepare to communicate over ROS */
        WirelessReceiver(const ros::NodeHandle& libscout_node);

        /** Register a callback to be called every time a packet is received */
        void register_callback(WirelessCallback new_callback);

    private:
        ros::Subscriber sub;
        ros::NodeHandle node;
        WirelessCallback callback;
        void receive(const ::messages::WirelessPacketConstPtr&);
        void dummy(std::vector<uint8_t> data);
};

#endif
