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
 * @file WirelessSender.cpp
 * @brief Contains wireless declarations and functions
 * 
 * @defgroup wirelesssender WirelessSender
 * @brief Contains functions and definitions for the use of XBee wireless
 * @ingroup behavior
 *
 * @author Colony Project, CMU Robotics Club
 * @author Tom Mullins
 *
 * @{
 **/

#include "WirelessSender.h"

using namespace std;
using namespace ::messages;

WirelessSender::WirelessSender(const ros::NodeHandle& libscout_node,
                               uint16_t type, uint16_t pan, uint16_t channel)
    : node(libscout_node)
{
    packet.type = type;
    packet.pan = pan;
    packet.channel = channel;
    pub = node.advertise< ::messages::WirelessPacket>("/wireless/send",
                                                      QUEUE_SIZE, true);
}

void WirelessSender::setType(uint16_t type)
{
    packet.type = type;
}
void WirelessSender::setPan(uint16_t pan)
{
    packet.pan = pan;
}
void WirelessSender::setChannel(uint16_t channel)
{
    packet.channel = channel;
}

void WirelessSender::send(uint8_t data[], int length, uint16_t dest)
{
    packet.dest = dest;
    packet.data.clear();
    for (int i = 0; i < length; i++)
    {
        packet.data.push_back(data[i]);
    }
    pub.publish(packet);
}

void WirelessSender::send(vector<uint8_t>& data, uint16_t dest)
{
    packet.dest = dest;
    packet.data = data;
    pub.publish(packet);
}

/** @} */
