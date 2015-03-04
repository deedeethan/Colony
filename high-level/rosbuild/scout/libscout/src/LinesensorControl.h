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
 * @file LinesensorControl.h
 * @brief Contains line follwing sensor declarations and functions
 *
 * @author Colony Project, CMU Robotics Club
 * @author Alex Zirbel
 */

#ifndef _LINESENSOR_CONTROL_H_
#define _LINESENSOR_CONTROL_H_

#include <ros/ros.h>
#include <messages/query_linesensor.h>

#include "constants.h"

class LinesensorControl
{
    public:
        /** Set up the encoders class and prepare to communicate over ROS */
        LinesensorControl(const ros::NodeHandle& libscout_node,
                          std::string scoutname);

        /** Use ROS to get the current readings. */
        std::vector<uint32_t> query();

        /** Get line position */
        double readline();

        /** Check if we are at an intersecetion */
        bool fullline(std::vector<uint32_t> readings);

        /** Check if we are at the destination for a maze
         *  (a special line pattern) */
        bool destination(std::vector<uint32_t> readings);

    private:
        /* ROS publisher and client declaration */
        ros::ServiceClient query_client;
        ros::NodeHandle node;
};

#endif /* _LINESENSOR_CONTORL_H_ */

