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

#include "wl_test.h"

using namespace std;

void wl_test::data_callback(std::vector<uint8_t> data)
{
    // We got a response! Stop publishing data = [1]
    if (no_response)
        no_response = false;

    ROS_INFO("Wireless callback triggered.");
    data[0]++;
    sleep(1);
    wl_sender->send(data,1);
}

void wl_test::run()
{
    wl_receiver->register_callback(std::bind(&wl_test::data_callback, this,
        std::placeholders::_1));
    while(ok())
    {
        // Publish 1 if we are scout1 until we get a response.
        if (scout_name == "scout1" && no_response) 
        {
            uint8_t data[1];
            data[0] = 1;
            wl_sender->send(data,1);
        }
        spinOnce();
        loop_rate->sleep();
    }
}
