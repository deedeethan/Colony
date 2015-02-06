/**
 * The code in this package was developed using the structure of Willow
 * Garage's turtlesim package.  It was modified by the CMU Robotics Club
 * to be used as a simulator for the Colony Scout robot.
 *
 * All redistribution of this code is limited to the terms of Willow Garage's
 * licensing terms, as well as under permission from the CMU Robotics Club.
 * 
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
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *    Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *    Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 * @file scout_constants.h
 *
 * @ingroup scoutsim
 * @{
 */

#ifndef _SCOUTSIM_SCOUT_CONSTANTS_
#define _SCOUTSIM_SCOUT_CONSTANTS_

#include <messages/set_motors.h>

/// @todo put these in a utility file somewhere?
#define min(x,y) ((x < y) ? x : y)
#define max(x,y) ((x > y) ? x : y)

namespace scoutsim
{
    // Maximum speed which will be sent to scoutsim in absolute units
    const int MAX_ABSOLUTE_SPEED = messages::set_motors::MAX_SPEED;
    const int MIN_ABSOLUTE_SPEED = messages::set_motors::MIN_SPEED;

    // Speed in m/s corresponding to maximum absolute speed
    const float MAX_SPEED_MPS = 1.0;

    // Pixels in scoutsim per real-world meter.
    // Note that scout image size has been set based on this.
    const float PIX_PER_METER = 200.0;

    // Scout dimensions, in meters
    const float SCOUT_WIDTH = 0.125;
    const float SCOUT_LENGTH = 0.23;

    // Position of sonar relative to robot center
    const float SCOUT_SONAR_X = -0.01;
    const float SCOUT_SONAR_Y = 0.0;

    // @todo Inaccurate. Update
    const float ENCODER_TICKS_PER_METER = 363.78;

    // @todo Update this
    const float SONAR_MAX_RANGE_M = 9.144;
    const unsigned int SONAR_MAX_RANGE_PIX = (unsigned int)
        (SONAR_MAX_RANGE_M * PIX_PER_METER);

    // Time it takes for the sonar to spin from position 0 to position 23
    const float SONAR_HALF_SPIN_TIME = 0.5;

    const float REAL_TIME_REFRESH_RATE = 0.05;   // s
    const float SIM_TIME_REFRESH_RATE = 0.02;   // s
}

#endif

/** @} */
