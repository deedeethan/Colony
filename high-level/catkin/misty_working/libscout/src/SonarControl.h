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
 * @file SonarControl.h
 * @brief Contains sonar declarations and functions
 * 
 * Contains functions and definitions for the use of
 * sonar
 *
 * @author Colony Project, CMU Robotics Club
 * @author Priyanka Deo
 **/

#ifndef _SONAR_CONTROL_H_
#define _SONAR_CONTROL_H_

#include <ros/ros.h>
#include <messages/sonar_set_scan.h>
#include <messages/sonar_toggle.h>
#include <messages/sonar_distance.h>

#include "constants.h"

class SonarControl
{
    public:
        /** Set up the motor node and prepare to communicate over ROS */
        SonarControl(const ros::NodeHandle& libscout_node,
                     std::string scoutname);

        /** Sets sonar to a position (0-23) specified by input */
        void set_single(int position);

        /** Sets sonar to scan a range in 0-23 specified by input */
        void set_range(int start_pos, int end_pos);

        /** (Re)starts sonar panning and taking readings */
        void set_on();

        /** Stops sonar from panning and taking readings */
        void set_off();	

        /** Get sonar readings */
        int* get_sonar_readings();

    private:
        /** Record the new sonar distance measurement */
        void distance_callback(const messages::sonar_distance::ConstPtr& msg);

        /** Sends a sonar_toggle message. */
        void set_power(bool is_on);

        /** Converts between values output by sensor and physical distances */
        //float sonar_to_dist(float sonar_value);
        //float dist_to_sonar(float distance);

        /** If a service call fails, tries again this many times. */
        int num_attempts;

        /** Keep track of the latest readings and their time of receipt.
         *  Readings are in millimeters. */
        int readings[48];
        ros::Time timestamps[48];

        /* ROS publisher and client declaration */
        ros::ServiceClient sonar_set_scan_client;
        ros::ServiceClient sonar_toggle_client;
        ros::Subscriber sonar_distance_sub;

        ros::NodeHandle node;
};

#endif

