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
 * @file Behavior.h
 * @brief Contains declarations for the structure of all behaviours.
 * 
 * Contains functions and definitions for a generic behavior.
 *
 * @author Colony Project, CMU Robotics Club
 * @author Priyanka Deo
 * @author Alex Zirbel
 **/

#ifndef _BEHAVIOR_H_
#define _BEHAVIOR_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

#include "MotorControl.h"
#include "HeadlightControl.h"
#include "ButtonControl.h"
#include "SonarControl.h"
//#include "CliffsensorControl.h"
#include "EncodersControl.h"
#include "LinesensorControl.h"
#include "WirelessSender.h"
#include "WirelessReceiver.h"
#include "constants.h"
#include "Sensors.h"

typedef ros::Time Time;
typedef ros::Duration Duration;

#define WAIT_HZ 100     // Frequency of spinOnce() calls in wait()

class Behavior
{
    public:
        // Initializes ROS for behavior
        // name stands for behavior name
        Behavior(std::string scoutname, std::string name,
                 Sensors * sensor);

        virtual ~Behavior();

        /// Extended by subclasses to actually run the behavior.
        virtual void run() = 0;

        // Name of behaviour
        std::string name;

        // Flag to check if sigint recieved.
        static bool keep_running;
        
    protected:
        ros::NodeHandle node;

        /// @todo Should this really be a pointer, or the object itself somehow?
        ros::Rate *loop_rate;

        // Declare all used library controls
        MotorControl * motors;
        ButtonControl * buttons;
        SonarControl * sonar;
        //CliffsensorControl * cliffsensor;
        EncodersControl * encoders;
        LinesensorControl * linesensor;
        WirelessSender * wl_sender;
        WirelessReceiver * wl_receiver;
        PaintBoardControl * paintboard;
        MetalDetectorControl * metal_detector;
        BomControl * bom;

        // Default callback for wireless receiver.
        void default_callback(std::vector<uint8_t> data);

        // Wrappers for ROS functions
        static bool ok();
        static void spin();
        static void spinOnce();

        virtual void wait(float duration);
};

#endif
