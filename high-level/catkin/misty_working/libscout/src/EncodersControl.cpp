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
 * @file EncodersControl.cpp
 * @brief Contains encoder function implementation
 *
 * @defgroup encoderscontrol EncodersControl
 * @brief Functions which a behavior can use to control the encoders.
 * @ingroup behavior
 * 
 * @author Colony Project, CMU Robotics Club
 * @author Alex Zirbel
 */

#include "EncodersControl.h"

using namespace std;

/**
 * @brief Initialize the encoders module of libscout.
 */
EncodersControl::EncodersControl(const ros::NodeHandle& libscout_node,
                                 string scoutname)
    : node(libscout_node)
{
    query_client = node.serviceClient< ::messages::query_encoders>(
            scoutname+"/query_encoders");
    reset_client = node.serviceClient< ::messages::reset_encoders>(
            scoutname+"/reset_encoders");
}

/**
 * @brief Returns the current tick count of the encoders.
 */
encoder_readings EncodersControl::query()
{
    // Set scan range
    ::messages::query_encoders srv;

    encoder_readings cur_readings;

    if (!query_client.call(srv))
    {
        ROS_ERROR("EncodersControl query failed.");
    }

    cur_readings.fl_ticks = srv.response.fl_distance;
    cur_readings.fr_ticks = srv.response.fr_distance;
    cur_readings.bl_ticks = srv.response.bl_distance;
    cur_readings.br_ticks = srv.response.br_distance;

    return cur_readings;
}

/**
 * @brief Resets the encoder tick count to 0.
 */
void EncodersControl::reset()
{
    ::messages::reset_encoders srv;
    if (!reset_client.call(srv))
    {
        ROS_ERROR("EncodersControl reset failed.");
    }
}

/** @} */
