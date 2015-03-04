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
 * @author Colony Project, CMU Robotics Club
 * @author Yuyang Guo
 */

#include "MetalDetectorControl.h"

using namespace std;

/**
 * @brief Initialize the Metal Detector module of libscout
 */
MetalDetectorControl::MetalDetectorControl(const ros::NodeHandle& libscout_node,
                string scoutname)
    : node(libscout_node)
{
    query_client = 
            node.serviceClient< ::messages::query_metal_detector>(
                    scoutname+"/query_metal_detector");
}

/**
 * @brief query the metal detector and return the value
 */
bool MetalDetectorControl::query()
{
    ::messages::query_metal_detector srv;

    if (!query_client.call(srv))
    {
        ROS_ERROR("MetalDetectorControl query failed.");
    }
    return srv.response.metal;
}
