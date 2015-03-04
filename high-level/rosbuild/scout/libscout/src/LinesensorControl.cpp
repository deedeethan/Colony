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
 * @file LinesensorControl.cpp
 * @brief Contains line following function implementation
 *
 * @defgroup linesensorcontrol LinesensorControl
 * @brief Functions which a behavior can use to control the line sensor.
 * @ingroup behavior
 * 
 * @author Colony Project, CMU Robotics Club
 * @author Alex Zirbel
 *
 * @{
 */

#include "LinesensorControl.h"

using namespace std;

static double last_ret;

/**
 * @brief Initialize the line following module of libscout.
 */
LinesensorControl::LinesensorControl(const ros::NodeHandle& libscout_node,
                                 string scoutname)
    : node(libscout_node)
{
    query_client =
        node.serviceClient< ::messages::query_linesensor>(scoutname+"/query_linesensor");
}

/**
 * @brief Returns the current readings of the line follwing sensor.
 */
vector<uint32_t> LinesensorControl::query()
{
    // Set scan range
    ::messages::query_linesensor srv;

    if (!query_client.call(srv))
    {
        ROS_ERROR("LinesensorControl query failed.");
    }

    if (srv.response.readings.size() != 8)
    {
        ROS_WARN("Linesensor reading vector has %d readings, 8 expected.",
                 int(srv.response.readings.size()));
    }

    return srv.response.readings;
}

/** 
 *  @brief Returns the average readings of the line following sensors. 
 */
double LinesensorControl::readline()
{
    unsigned int total_read = 0;
    unsigned int weighted_total = 0;

    vector<uint32_t> readings = query();

    if (readings.size() < 8)
    {
        return last_ret;
    }

    for (int i = 0; i < 8; i++)
    {
        total_read += readings[i];
        weighted_total += i * readings[i];
    }

    if (total_read == 0)
    {
        return last_ret;
    }
    
    double ret_val = last_ret = ((double) weighted_total / total_read) - 3.5;
    return ret_val;
}

bool LinesensorControl::fullline(vector<uint32_t> readings)
{
    if (readings.size() < 8)
    {
        return false;
    }

    for(int i=0; i<8; i++)
    {
        if(readings[i] < 200)
          return false;
    }
    return true; 
}

/**
 * We define a destination as two gaps of white between the line and patches
 * of 95-shade color (dark grey) on either side.
 * I use 95-shade color in an attempt to not throw off the line localization.
 */
bool LinesensorControl::destination(vector<uint32_t> readings)
{
    if (readings.size() < 8)
    {
        return false;
    }

    // Try to match this pattern to what we see
    int expected_pattern[] = {95, 0, 255, 0, 95};
    int pat_idx = 0;

    for (int i = 0; i < 8; i++)
    {
        // If the difference between what we have and expect is less than 5
        if (abs(readings[i] - expected_pattern[pat_idx]) < 5)
        {
            // Check. Keep looking for the rest of the pattern.
            pat_idx++;
        }

        // The whole pattern was found
        if (pat_idx > 4)
        {
            return true;
        }
    }
    return false;
}

/** @} */
