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
 * @brief Contains basic functions for the structure of all behaviors.
 * 
 * Contains function implementations needed for all behavior.
 *
 * @author Colony Project, CMU Robotics Club
 * @author Priyanka Deo
 * @author Alex Zirbel
 **/

#include "Sensors.h"

using namespace std;

Sensors::Sensors(string scoutname)
{
    name = scoutname;
    motors = new MotorControl(node,scoutname);
    buttons = new ButtonControl(node, scoutname);
    sonar = new SonarControl(node, scoutname);
    //cliffsensor = new CliffsensorControl(node, scoutname);
    encoders = new EncodersControl(node, scoutname);
    linesensor = new LinesensorControl(node, scoutname);
    wl_sender = new WirelessSender(node);
    paintboard = new PaintBoardControl(node, scoutname);
    metal_detector = new MetalDetectorControl(node, scoutname);
    bom = new BomControl(node, scoutname);
}

// b_stuff stand for behavior control class
void Sensors::init(MotorControl** b_motors, ButtonControl ** b_buttons,
                  SonarControl ** b_sonar, EncodersControl ** b_encoders,
                  LinesensorControl ** b_linesensor,
                  WirelessSender ** b_wl_sender,
                  WirelessReceiver ** b_wl_receiver,
                  PaintBoardControl ** b_paintboard,
                  MetalDetectorControl ** b_metal_detector,
                  BomControl ** b_bom)
{
    //(MotorControl *)(* b_motors) = motors;
    *b_motors = motors;
    *b_buttons = buttons;
    *b_sonar = sonar;
    *b_encoders = encoders;
    *b_linesensor = linesensor;
    *b_wl_sender = wl_sender;
    *b_wl_receiver = new WirelessReceiver(node);
    *b_paintboard = paintboard;
    *b_metal_detector = metal_detector;
    *b_bom = bom;
}
