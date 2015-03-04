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
 * @file encoders.cpp
 * @brief Encoders
 *
 * @defgroup encoders Encoders
 * @brief Functions for using the encoders
 *
 * @author Colony Project, CMU Robotics Club
 * @author Alex Zirbel
 * @author Tom Mullins

 * @{
 **/

#include <fstream>
#include <ros/ros.h>
#include "encoders.h"

/* Encoder instances */
/* @todo make sure these are the correct numbers */
static Encoder encoder_fl(0);
static Encoder encoder_fr(1);
static Encoder encoder_bl(2);
static Encoder encoder_br(3);

/**
 * @brief Encoder constructor
 *
 * Opens device file, which is read at every call to get_ticks
 *
 * @param n The encoder number to read, between 0 and 3 inclusive
 */
Encoder::Encoder(int n)
{
    sprintf(filename, "/sys/class/encoder/enc%d/ticks", n);
}

/**
 * @brief Returns the current tick count
 *
 * This will actually read from the encoder ticks file to get the latest value
 * from the driver
 */
int Encoder::get_ticks()
{
    // open device file
    std::fstream fticks(filename, std::ios::in);

    int ticks;
    fticks >> ticks;

    return ticks;
}

/**
 * @brief Resets the encoder
 *
 * Sets the encoder's ticks value back to 0
 */
void Encoder::reset()
{
    std::fstream fticks(filename, std::ios::out);
    fticks << 0;
}

/**
 * @brief Outputs current encoder data
 *
 * Serves the service query_encoders by responding to service requests with the
 * encoder values.
 * @param req The request. The units that the response should be in.
 * @param res The response. The fields will be filled with values.
 */
bool handle_encoders_query(::messages::query_encoders::Request  &req,
                           ::messages::query_encoders::Response &res)
{
    /* Put index, velocity, and distance data in message */
    res.fl_distance = -encoder_fl.get_ticks();
    res.fr_distance = encoder_fr.get_ticks();
    res.bl_distance = encoder_bl.get_ticks();
    res.br_distance = encoder_br.get_ticks();

    /* @todo maybe return value based on whether reads succeeded */

    ROS_DEBUG("Encoder values queried");
    ROS_INFO("Encoder values queried %d  %d  %d   %d", res.fl_distance, res.fr_distance, res.bl_distance, res.br_distance);
    return true;
}

/**
 * @brief Resets all four encoders
 *
 * Serves the service reset_encoders
 * @param req The request. Empty.
 * @param res The response. Empty.
 */
bool handle_reset_query(::messages::reset_encoders::Request &req,
                        ::messages::reset_encoders::Response &res)
{
    encoder_fl.reset();
    encoder_fr.reset();
    encoder_bl.reset();
    encoder_br.reset();
    return true;
}

/**
 * @brief Encoders driver. This is a ROS node that controls encoders.
 *
 * This is the main function for the encoders node. It is run when the node
 * starts and initializes the encoders. It then publishes to the
 * encoder_state topic and advertises the query_encoders service.
 */
int main(int argc, char **argv)
{
    /* Initialize in ROS the encoders driver node */
    ros::init(argc, argv, "encoders_driver");

    /* Advertise that this serves the query_encoders service */
    ros::NodeHandle n;
    ros::ServiceServer service =
        n.advertiseService("query_encoders",
                           handle_encoders_query);

    ROS_INFO("Ready to set encoders.");

    ros::spin();

    return 0;
}

/** @} */
