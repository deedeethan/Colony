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
 **/

/**
 * @file CliffsensorControl.h
 * @brief Contains cliffsensor declarations and functions.
 * 
 * Contains functions and definitions for the use of
 * cliffsensors.
 *
 * @author Colony Project, CMU Robotics Club
 * @author Priyanka Deo
 * @author Leon Zhang
 **/

#ifndef _CLIFFSENSOR_H_
#define _CLIFFSENSOR_H_

#include <messages/query_cliff.h>
#include <messages/cliff_status_changed.h>
#include "constants.h"

#define CLIFF_DETECTED 0x0
#define NO_CLIFF 0x1

class CliffsensorControl
{
    public:
        CliffsensorControl(const ros::NodeHandle& libscout_node, std::string scoutname);

        /** @brief Responds to topic to change cliff sensor status. **/
        void changed_cliff_status(
                        const messages::cliff_status_changed::ConstPtr& msg);

        int get_front_raw();
        int get_left_raw();
        int get_right_raw();
        bool check_is_cliff();

    private:
        /* Cliffsensor state variables
         */
        int front_raw; /**< The current raw value 
                        data of the front cliffsensor. */
        int left_raw; /**< The current raw value data of 
                        the left cliffsensor. */
        int right_raw; /**< The current raw value data of 
                        the right cliffsensor. */

        ros::Subscriber cliff_status_changed_sub;
        ros::NodeHandle node; 
};

#endif
