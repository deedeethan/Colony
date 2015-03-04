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

#include "danger_marking.h"

#define THRESH 200
#define min(x,y) ((x < y) ? x : y)

using namespace std;

void danger_marking::run()
{    
    ROS_INFO("Starting Danger Marking.");

    // Turn the sonar on.
    sonar->set_on();
    wait(1.5);
    sonar->set_range(35, 37);

    motors->set_sides(-20, -20);

    int dist;

    while (true)
    {
        get_position();
        ROS_INFO("Position: (%f, %f, %f).",
            scout_pos->x, scout_pos->y, scout_pos->theta);

        int *readings = sonar->get_sonar_readings();
        dist = min(readings[35], min(readings[36], readings[37]));

        ROS_INFO("Dist: %d.", dist);

        if (dist < THRESH)
        {
            ROS_INFO("UH-OH! Get back!");
            motors->set_sides(30, 30);
            wait(0.5);
            motors->set_sides(30, -30);
            wait(1.0);
        }
        else
        {
            motors->set_sides(-20, -20);
            if (metal_detector->query()) //FOUND SOME METAL!
            {
                paintboard->set(DISPENSE_PAINT);
            }
            else
            {
                if(paintboard->query()) //No metal, but dispensing paint;
                {
                    paintboard->set(STOP_PAINT);
                }
            }
        }
    }
}
