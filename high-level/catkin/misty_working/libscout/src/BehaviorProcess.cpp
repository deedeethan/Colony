
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
 * @file Behavior.cpp
 * @brief A wrapper that executes PriyaBehavior as an executable.
 *
 * Should be auto-generated in the future.
 * 
 * @author Colony Project, CMU Robotics Club
 * @author Alex Zirbel
 */

#include "BehaviorProcess.h"
#include "Sensors.h"
#include <assert.h>
#include <signal.h>

// Make the behavior's ok() function return false.
void sigint_handler(int sig)
{
  Behavior::keep_running = false;  
}

int main (int argc, char **argv)
{
    string scoutname = "";
    int behavior_num;
    string behavior_name = "";

    // Running with no arguments only supports one scout. Check in case
    // the user meant to specify a scout in the arguments.
    if (argc < 2)
    {
        cout << "You have started this behavior in hardware mode." << endl
             << "To start in software mode, use: " << argv[0]
             << " <behavior#> <scoutname> " << endl;
    }
    else
    {
        // Use the provided scoutname for simulator messages
        //scoutname = argv[1];
        behavior_num = atoi(argv[1]);
        if (argc == 3) {
            scoutname = argv[2];
        }

        ros::init(argc, argv, scoutname + "Behavior",
            ros::init_options::NoSigintHandler);

        // Initialize the signal handler after initializing rosnode.
        // Otherwise it will be overwritten and you will be sad.
        signal(SIGINT, sigint_handler);

        // one Sensor instance per-class
        Sensors* sensors = new Sensors(scoutname);
        BehaviorList* list = new BehaviorList();
        vector<behavior_func> behavior_list = list->behavior_list;
        if (behavior_num < (int)behavior_list.size())
        {
            Behavior* b = behavior_list[behavior_num](scoutname, sensors);
            b->run();
            delete b;
        }
        else
        {
            cout << "There is no behavior number" << behavior_num
              << ". There are only " << (int)behavior_list.size() << "behaviors."
              << endl;
        }

        delete list;
        delete sensors;
    }

    return 0;
}
