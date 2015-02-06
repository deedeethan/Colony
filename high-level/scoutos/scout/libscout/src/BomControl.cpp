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
 * @file BomControl.h
 * @brief Contains line follwing sensor declarations and functions
 *
 * @author Colony Project, CMU Robotics Club
 * @author Yuyang (Misty) Guo
 */

 #include "BomControl.h"

 using namespace std;

/**
 * @brief Initialize the BOM module of libscout.
 */
BomControl::BomControl(const ros::NodeHandle& libscout_node, string scoutname)
    : node(libscout_node)
{
    query_client =
        node.serviceClient< ::messages::query_boms>(scoutname+"/query_BOM");
}

BomReadings BomControl::query()
{
    ::messages::query_boms srv;

    if (!query_client.call(srv))
    {
        ROS_ERROR("BOM_Control query failed.");
    }

    if (srv.response.readings.size() != 10)
    {
        ROS_WARN("BOM_Control reading vector has %d readings, 10 expected.",
                 int(srv.response.readings.size()));
    }

    BomReadings result;
    result.readings = srv.response.readings;
    result.senders = srv.response.senders;
    return result;
}

