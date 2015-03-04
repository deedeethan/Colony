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

#ifndef _LINE_FOLLOW_H_
#define _LINE_FOLLOW_H_

#include "../Behavior.h"
#include "../Sensors.h"

#define MOTOR_BASE 40.0f
#define KP 20.0f

class line_follow : public Behavior
{
    public:
        line_follow(std::string scoutname, std::string behavior_name, Sensors* sensors) : 
                Behavior(scoutname, behavior_name, sensors) {};

        line_follow(std::string scoutname, Sensors* sensors) : 
                Behavior(scoutname, "line_follow", sensors) {};

        /** Actually executes the behavior. */
        void run();

    protected:
        void turn_left();
        void turn_right();
        void turn_straight();
        void spot_turn();
        void u_turn();
        void follow_line();
        void halt();
};

#endif
