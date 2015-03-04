/**
 * Copyright (c) 2013 Colony Project
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
 * @file painter.cpp
 * @brief Contains code to control the painter and query the metal detector.
 *
 * @author Colony Project, CMU Robotics Club
 * @author Tom Mullins
 *
 */

#define QUEUE_SIZE 10

#include <cstdlib>
#include <ros/ros.h>
#include <messages/set_paintboard.h>
#include <messages/query_paintboard.h>
#include <messages/query_metal_detector.h>
#include "paint-i2c.h"

using namespace std;

int painter_value;

/**
 * @brief ROS callback to turn the paint output on or off
 */
void paint_set(const ::messages::set_paintboard::ConstPtr& msg)
{
  painter_value = msg->setting;
  set_servo(0, painter_value);
}

/**
 * @brief ROS callback to query whether the paint output is on or off
 */
bool paint_query(::messages::query_paintboard::Request &req,
                 ::messages::query_paintboard::Response &res)
{
  res.setting = painter_value;
  return true;
}

/**
 * @brief ROS callback to query whether there is metal detected
 */
bool metal_query(::messages::query_metal_detector::Request &req,
                 ::messages::query_metal_detector::Response &res)
{
  res.metal = get_input(0);
  return true;
}

/**
 * @brief Painter driver for ROS, to control the painter and metal detector
 *
 * @param argc The number of command line arguments (should be 1)
 * @param argv The array of command line arguments
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "painter");
    ros::NodeHandle node;

    i2c_start();
    set_motor(0, 0);
    set_servo(0, 0);

    ros::Subscriber painter_sub = node.subscribe("/set_paintboard", QUEUE_SIZE,
        paint_set);
    ros::ServiceServer painter_srv = node.advertiseService("/query_paintboard",
        paint_query);
    ros::ServiceServer metal_srv = node.advertiseService("/query_metal_detector",
        metal_query);

    ROS_INFO("Painter node ready.");
    ros::spin();

    i2c_stop();

    return 0;
}
