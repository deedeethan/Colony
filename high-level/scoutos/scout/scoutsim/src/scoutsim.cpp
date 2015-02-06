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
 *
 * @defgroup scoutsim Scoutsim
 * @{
 */

#include <wx/app.h>
#include <wx/timer.h>

#include <ros/ros.h>

#include <boost/thread.hpp>

#include <stdio.h>

#include "sim_frame.h"

#ifdef __WXMAC__
#include <ApplicationServices/ApplicationServices.h>
#endif

using namespace std;

class ScoutApp : public wxApp
{
    public:
        char** local_argv;

        ScoutApp()
        {
        }

        bool OnInit()
        {
            std::cout << "Starting scout app." << std::endl;
#ifdef __WXMAC__
            ProcessSerialNumber PSN;
            GetCurrentProcess(&PSN);
            TransformProcessType(&PSN,kProcessTransformToForegroundApplication);
            SetFrontProcess(&PSN);
#endif
            // Create our own copy of argv, with regular char*s.
            local_argv = new char*[ argc ];
            for ( int i = 0; i < argc; ++i )
            {
                local_argv[ i ] = strdup( wxString( argv[ i ] ).mb_str() );
            }

            // Check for incorrect usage
            if (argc < 2)
            {
                cout << endl << "Error." << endl << endl;
                cout << "Usage: " << local_argv[0] << " <map name>" << endl;
                cout << "To use maps/example.bmp, use 'example'." << endl;
                exit(0);
            }

            std::cout << "About to init node" << std::endl;
            ros::init(argc, local_argv, "scoutsim");
            std::cout << "About to reset." << std::endl;

            std::cout << "About to init image handlers." << std::endl;
            wxInitAllImageHandlers();

            std::cout << "About to make a sim frame." << std::endl;
            scoutsim::SimFrame* frame = new scoutsim::SimFrame(NULL, string(local_argv[1]));

            SetTopWindow(frame);
            frame->Show();

            std::cout << "Constructor done!" << std::endl;
            return true;
        }

        int OnExit()
        {
            for ( int i = 0; i < argc; ++i )
            {
                free( local_argv[ i ] );
            }
            delete [] local_argv;

            return 0;
        }
};

BEGIN_EVENT_TABLE(scoutsim::SimFrame, wxFrame)
    EVT_MENU(ID_QUIT,  scoutsim::SimFrame::onQuit)
    EVT_MENU(ID_ABOUT, scoutsim::SimFrame::onAbout)
    EVT_MENU(ID_CLEAR, scoutsim::SimFrame::onClear)
    EVT_MENU(ID_MAP, scoutsim::SimFrame::showMap)
    EVT_MENU(ID_LINES, scoutsim::SimFrame::showLines)
    EVT_MENU(ID_WALLS, scoutsim::SimFrame::showWalls)
    EVT_MENU(ID_TELEOP_NONE, scoutsim::SimFrame::stopTeleop)
    EVT_MENU(ID_TELEOP_PRECISE, scoutsim::SimFrame::startTeleopPrecise)
    EVT_MENU(ID_TELEOP_FLUID, scoutsim::SimFrame::startTeleopFluid)
END_EVENT_TABLE()

DECLARE_APP(ScoutApp);
IMPLEMENT_APP(ScoutApp);

/** @} */
