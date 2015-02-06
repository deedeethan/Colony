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
 * @file WirelessSender.h
 * @brief Contains wireless declarations and functions
 * 
 * Contains functions and definitions for the use of
 * XBee wireless
 *
 * Uses the dest field of the wireless packet to specify which
 * scout the message is intended for, or SCOUT_ALL to send to
 * all scouts
 *
 * @author Colony Project, CMU Robotics Club
 * @author Tom Mullins
 **/

#ifndef _WIRELESS_SENDER_H_
#define _WIRELESS_SENDER_H_

#include <vector>
#include <ros/ros.h>
#include "messages/WirelessPacket.h"
#include "constants.h"

/// A macro to allow sending to all the scouts.
#define SCOUT_ALL (messages::WirelessPacket::DEST_BROADCAST)

class WirelessSender
{
    public:
        WirelessSender(const ros::NodeHandle& libscout_node,
                uint16_t type = 0,
                uint16_t pan = messages::WirelessPacket::PAN_DEFAULT,
                uint16_t channel = messages::WirelessPacket::CHANNEL_DEFAULT);
        
        void setType(uint16_t type);
        void setPan(uint16_t pan);
        void setChannel(uint16_t channel);
        
        void send(uint8_t data[], int length, uint16_t dest = SCOUT_ALL);
        void send(std::vector<uint8_t>& data, uint16_t dest = SCOUT_ALL);
        
    private:
        ros::NodeHandle node;
        ros::Publisher pub;
        messages::WirelessPacket packet;
};

#endif
