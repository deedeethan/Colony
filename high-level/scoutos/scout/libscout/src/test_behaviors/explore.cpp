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

#include "explore.h"

using namespace std;


void explore::run()
{
    ROS_INFO("Starting to explore region");
    sonar->set_on();
    sonar->set_range(0, 23);
    Duration sleeptime(1);
    while (ok()) {
        int *readings = sonar->get_sonar_readings();
        // Look to the left.
        int left_distance = readings[0];
        // Look to the front.
        int front_distance = readings[12];
        // Look to the right.
        int right_distance = readings[24];
        // Look to the back
        int back_distance = readings[36];
        ROS_INFO("front: %d  left: %d  back: %d  right: %d ", front_distance, left_distance, back_distance, right_distance);
        sleeptime.sleep();
        spinOnce();
    }
}
