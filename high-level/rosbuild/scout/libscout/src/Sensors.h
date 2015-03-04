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

#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "MotorControl.h"
#include "HeadlightControl.h"
#include "ButtonControl.h"
#include "BomControl.h"
#include "SonarControl.h"
//#include "CliffsensorControl.h"
#include "EncodersControl.h"
#include "LinesensorControl.h"
#include "WirelessSender.h"
#include "WirelessReceiver.h"
#include "PaintBoardControl.h"
#include "MetalDetectorControl.h"

class Sensors
{
    public:
        Sensors(std::string scoutname);
        // b_stuff stand for behavior control class
        void init(MotorControl ** b_motors, ButtonControl ** b_buttons,
                  SonarControl ** b_sonar, EncodersControl ** b_encoders,
                  LinesensorControl ** b_linesensor,
                  WirelessSender ** b_wl_sender,
                  WirelessReceiver ** b_wl_receiver,
                  PaintBoardControl ** b_paintboard,
                  MetalDetectorControl ** b_metal_detector,
                  BomControl ** b_bom
                  );
        std::string name;
    private:
        ros::NodeHandle node;
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
};


#endif
