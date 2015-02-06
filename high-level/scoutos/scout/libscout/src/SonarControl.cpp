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
 * @file SonarControl.cpp
 * @brief Contains sonar function implementation
 * 
 * @defgroup sonarcontrol SonarControl
 * @brief Functions which a behavior can use to control the sonar.
 * @ingroup behavior
 *
 * @author Colony Project, CMU Robotics Club
 * @author Priyanka Deo
 * @author Alex Zirbel
 *
 * @{
 **/

#include "SonarControl.h"

#define MAX_ATTEMPTS 10

using namespace std;

/**
 * @brief Initialize the sonar module of libscout.
 *
 * Initialize the libscout node as a publisher of sonar_set_scan and 
 * sonar_toggle and a client of query_sonar.
 */
SonarControl::SonarControl(const ros::NodeHandle& libscout_node,
                           string scoutname)
    : node(libscout_node)
{
    sonar_set_scan_client =
        node.serviceClient< ::messages::sonar_set_scan>(scoutname+"/set_sonar_scan");
    sonar_toggle_client = 
        node.serviceClient< ::messages::sonar_toggle>(scoutname + "/toggle_sonar");
    sonar_distance_sub = node.subscribe(scoutname + "/sonar_distance",
                                        QUEUE_SIZE,
                                        &SonarControl::distance_callback,
                                        this);

    // Initialize all the sonar readings to 0 and timestamps to undefined
    for (int i = 0; i < 48; i++)
    {
        readings[i] = 0;
        timestamps[i] = ros::Time::now(); // A common shortcut for no reading
    }

    ROS_INFO("Readings at 0, ready to go.");

    /// @todo Test, and replace 10 with a const int
    num_attempts = MAX_ATTEMPTS;
}

/**
 * Update the array of sonar values, and the last read timestamps,
 * to reflect the new reading received.
 */
void SonarControl::distance_callback(const ::messages::sonar_distance::ConstPtr& msg)
{
    // Error checking so that the array doesn't cause a segfault
    if (msg->pos < 0 || msg-> pos > 23)
    {
        ROS_ERROR("SonarControl received an invalid sonar position.");
        return;
    }

    readings[msg->pos] = msg->distance0;
    readings[msg->pos + 24] = msg->distance1;

    timestamps[msg->pos] = msg->stamp;
    timestamps[msg->pos + 24] = msg->stamp;
    

}

/**
 * @brief Sets the sonar to a position.
 *
 * @param position Value between 0-180 of degree position to set sonar
 */
void SonarControl::set_single(int position)
{
    set_range(position, position);
}

/**
 * @brief Sets the sonar to scan between two positions
 *
 * @param start_pos The leftmost (smallest) value that the sonar can take
 * @param end_pos The rightmost (largest) value that the sonar can take
 */
void SonarControl::set_range(int start_pos, int end_pos)
{
    // Check that the range is valid
    if (start_pos < 0 || end_pos < 0 || start_pos > 23 || end_pos > 23)
    {
        ROS_ERROR("Commanded SonarControl::set_range to a bad value.");
        return;
    }

    // Sort the start and end positions into increasing order
    if (start_pos > end_pos)
    {
        int temp = start_pos;
        start_pos = end_pos;
        end_pos = temp;
    }

    // Make sure sonar is on
    set_on();

    // Set scan range
    ::messages::sonar_set_scan srv;
    srv.request.stop_l = start_pos;
    srv.request.stop_r = end_pos;

    // Check if the service call failed or if the response was false
    if (!sonar_set_scan_client.call(srv) || srv.response.ack == false)
    {
        if (--num_attempts == 0)
        {
            ROS_ERROR("SonarControl::set_range() failed permanently.");
        }

        ROS_WARN("SonarControl::set_range() failed once.");
    }
}

/**
 * @brief Turn on sonar readings
 *
 * (Re)starts sonar panning and taking readings.
 */
void SonarControl::set_on()
{
    set_power(true);
}

/**
 * @brief Turn off sonar readings
 *
 * Stops sonar from panning and taking readings.
 */
void SonarControl::set_off()
{
    set_power(false);
}

/**
 * @brief Turn sonar readings either off or on.
 *
 * Attempts to turn the readings off or on, num_attempts times.
 *
 * @param is_on True if power should be set on, false if should be off.
 */
void SonarControl::set_power(bool is_on)
{
    ::messages::sonar_toggle srv;
    srv.request.set_on = is_on;

    // Check if the service call failed or if the response was false
    if (!sonar_toggle_client.call(srv) || srv.response.ack == false)
    {
        if (--num_attempts == 0)
        {
            ROS_ERROR("SonarControl::set_power() failed permanently.");
        }

        ROS_WARN("SonarControl::set_power() failed once.");
    }
    ROS_INFO("SonarControl::set_power() succeeded.");

    // Reset num_attempts
    num_attempts = MAX_ATTEMPTS;
}

int* SonarControl::get_sonar_readings()
{
  return readings;
}

/**
 * @brief Converts value returne by sonar to physical distances.
 *
 * @param sonar_value The returned value of the sonar
 * @return The physical distance measured by the sonar.
 **/
//float sonar_to_dist(float sonar_value)
//{
    //@todo impelement later based on sonar readings
//    return sonar_value;
//}

/**
 * @brief Converts values from physical distances to values read by sonar
 *
 * @param distance The physical distance as measured.
 * @return The value read by the sonar that corresponds to the given distance
 **/
//float dist_to_sonar(float distance)
//{
    //@todo implement later based on sonar readings
//    return distance;
//}

/** @} */
