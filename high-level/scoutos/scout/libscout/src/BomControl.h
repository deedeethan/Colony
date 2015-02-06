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

#ifndef _BOM_CONTROL_H_
#define _BOM_CONTROL_H_

#include <ros/ros.h>
#include <messages/query_boms.h>

typedef struct BomReadings {
    std::vector<uint32_t> readings;
    std::vector<uint32_t> senders;
} BomReadings;

class BomControl
{
    public:
        /** Set up the encoders class and prepare to communicate over ROS */
        BomControl(const ros::NodeHandle& libscout_node,
                          std::string scoutname);

        /** Use ROS to get the current readings. */
        BomReadings query();

        /**  */
        //std::double getSourceEstimation();

    private:
        /* ROS publisher and client declaration */
        ros::ServiceClient query_client;
        ros::NodeHandle node;
};

#endif /* _BOM_CONTORL_H_ */

