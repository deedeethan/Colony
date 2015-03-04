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
 * @file encoders.h
 * @brief Contains encoder declarations and functions.
 * 
 * @author Colony Project, CMU Robotics Club
 * @author Alex Zirbel
 * @author Tom Mullins
 **/

#ifndef _ENCODERS_H_
#define _ENCODERS_H_

#include <messages/query_encoders.h>
#include <messages/reset_encoders.h>

class Encoder
{
    public:
        Encoder(int n);
        int get_ticks();
        void reset();
    private:
        char filename[60];
};

int main(int argc, char **argv);

bool handle_encoders_query(messages::query_encoders::Request  &req,
                           messages::query_encoders::Response &res);

#endif
