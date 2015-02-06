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

#ifndef _DRAW_MISTY_NAME_H_
#define _DRAW_MISTY_NAME_H_

#include "../../Behavior.h"

/** 
 * The following class inherits the behavior class. This
 * means that all functions and variables defined
 * in the Behavior class also apply to this class.
 */
class draw_misty_name : Behavior
{
    public:
        /** 
         * The following line is the constructor of this class.
         * It first calls the Behavior class's constructor with some
         * parameters and then does nothing on its own.
         */
        draw_misty_name(std::string scoutname, Sensors* sensors)
            : Behavior(scoutname, "draw_misty_name", sensors) {};

        /** This is the function that when called actually executes
         * the behavior. */
        void run();
    private:
        void turnRight();
};

#endif
