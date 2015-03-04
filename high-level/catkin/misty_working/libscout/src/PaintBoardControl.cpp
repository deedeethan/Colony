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
 * @file PaintBoardControl.cpp
 * @brief Contains paintboard declarations and functions
 * 
 * Contains functions and definitions for the use of
 * paintboards
 *
 * @author Colony Project, CMU Robotics Club
 * @author Priyanka Deo
 **/

#include "PaintBoardControl.h"

using namespace std;

/**
 * @brief Initialize the paintboards module of libscout.
 *
 * Initialize the libscout node as a publisher of set_paintboards and a client of
 * query_paintboards.
 */
PaintBoardControl::PaintBoardControl(const ros::NodeHandle& libscout_node,
                           string scoutname)
    : node(libscout_node)
{
    set_paintboard_pub =
        node.advertise< ::messages::set_paintboard>(scoutname + "/set_paintboard",
                                           QUEUE_SIZE, true);
    query_paintboard_client =
        node.serviceClient< ::messages::query_paintboard>(scoutname + "/query_paintboard");
}

PaintBoardControl::~PaintBoardControl()
{
  set(STOP_PAINT);
}

/**
 * @brief Sets all the speeds according to the specificiation of which paintboard
 *        to set.
 *
 * @param which A bitmask of which paintboard need to be set.
 * @param speed The value to set the paintboard to.
 * @param units The units the speed is expressed in.
 * @return Function status
 */
void PaintBoardControl::set(int paintboard_setting)
{
    ::messages::set_paintboard msg;
    
    msg.setting = paintboard_setting;

    set_paintboard_pub.publish(msg);
    ros::spinOnce();
}

/**
 * @brief Query the current speeds of the paintboard
 *
 * Sends a request to the query_paintboard service which will reply with the
 *  current setting of the paintboard.
 *
 * @param which A bitmask that will specify which motor speed should be
 *  returned
 * @return The speed of the selected motor
 */
int PaintBoardControl::query()
{
    ::messages::query_paintboard srv;
    if(query_paintboard_client.call(srv))
    {
      return srv.response.setting;
    }
    else
    {
        ROS_FATAL("Failed to call service query_paintboard");
    }

    return -1;
}
