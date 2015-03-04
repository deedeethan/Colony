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

#include "maze_solve_simple.h"

#define D_THRESH 600
#define max(x,y) ((x > y) ? x : y)

using namespace std;

void maze_solve_simple::run()
{    
    ROS_INFO("Starting to solve the maze");

    // Go up to the first line.
    follow_line();

    // Turn the sonar on.
    sonar->set_on();
    sonar->set_range(0, 23);

    while (!at_destination())
    {
        // Wait for sonar to update.
        wait(1.5);
        int* readings = sonar->get_sonar_readings();
        
        // Wait until the sonar gives us real values.
        bool readings_ok = true;
        for (int i = 0; i < 48; i++)
        {
            if (readings[i] == 0)
            {
                readings_ok = false;
                break;
            }
        }
        if (!readings_ok)
        {
            continue;
        }

        int r_read = max(readings[23], max(readings[24], readings[25]));
        int s_read = max(readings[35], max(readings[36], readings[37]));
        int l_read = max(readings[47], max(readings[0],  readings[1]));

        if (r_read > D_THRESH)      // Right
        {
            ROS_INFO("Right.");
            turn_right();
        }
        else if (s_read > D_THRESH) // Straight
        {
            ROS_INFO("Straight.");
            turn_straight();
        }
        else if (l_read > D_THRESH)   // Left
        {
            ROS_INFO("Left.");
            turn_left();
        }
        else                                                        // Deadend
        {
            ROS_INFO("Dead end.");
            spot_turn();
        }

        follow_line();
    }

    ROS_INFO("I have solved the maze!");
}

bool maze_solve_simple::at_destination() 
{
    vector<uint32_t> readings = linesensor->query();
    if ( readings[0] > 200 &&
         readings[1] < 55 &&
         readings[2] < 55 &&
         readings[3] > 200 &&
         readings[4] > 200 &&
         readings[5] < 55 &&
         readings[6] < 55 &&
         readings[7] > 200 )
    {
        return true;
    }
    return false;
}
