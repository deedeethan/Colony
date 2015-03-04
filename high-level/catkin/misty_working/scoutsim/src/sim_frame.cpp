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
 * @file sim_frame.cpp
 *
 * @ingroup scoutsim
 * @{
 */

#include "sim_frame.h"

#include <stdio.h>

#include <ros/package.h>
#include <cstdlib>
#include <ctime>

using namespace std;

namespace scoutsim
{
    SimFrame::SimFrame(wxWindow* parent, string map_name)
        : wxFrame(parent, wxID_ANY, wxT("ScoutSim"), wxDefaultPosition,
                  wxSize(500, 500), wxDEFAULT_FRAME_STYLE & ~wxRESIZE_BORDER)
          , frame_count(0)
          , id_counter(0)
    {
        std::cout << "Constructing sim frame." << std::endl;

        srand(time(NULL));

        update_timer = new wxTimer(this);
        update_timer->Start(REAL_TIME_REFRESH_RATE * 1000);

        Connect(update_timer->GetId(), wxEVT_TIMER,
                wxTimerEventHandler(SimFrame::onUpdate), NULL, this);
        Connect(wxEVT_PAINT, wxPaintEventHandler(SimFrame::onPaint),
                NULL, this);

        images_path = ros::package::getPath("scoutsim") + "/images/";

        map_base_name =  ros::package::getPath("scoutsim") + "/maps/" +
                           map_name + ".bmp";
        map_lines_name = ros::package::getPath("scoutsim") + "/maps/" +
                           map_name + "_lines.bmp";
        map_walls_name = ros::package::getPath("scoutsim") + "/maps/" +
                           map_name + "_walls.bmp";
        display_map_name = map_base_name;

        wxBitmap lines_bitmap;
        wxBitmap walls_bitmap;
	ROS_INFO("Loading map: %s", display_map_name.c_str());
        path_bitmap.LoadFile(wxString::FromAscii(display_map_name.c_str()));

        // Try to load the file; if it fails, make a new blank file
        if (!lines_bitmap.LoadFile(wxString::FromAscii(map_lines_name.c_str())))
        {
            lines_bitmap = wxBitmap(path_bitmap.GetWidth(), path_bitmap.GetHeight(), 3);
        }
        lines_image = lines_bitmap.ConvertToImage();

        // Try to load the file; if it fails, make a new blank file
        if (!walls_bitmap.LoadFile(wxString::FromAscii(map_walls_name.c_str())))
        {
            walls_bitmap = wxBitmap(path_bitmap.GetWidth(), path_bitmap.GetHeight(), 3);
        }
        walls_image = walls_bitmap.ConvertToImage();

        clear();

        clear_srv = nh.advertiseService("clear",
                                        &SimFrame::clearCallback, this);
        reset_srv = nh.advertiseService("reset",
                                        &SimFrame::resetCallback, this);
        spawn_srv = nh.advertiseService("spawn",
                                        &SimFrame::spawnCallback, this);
        kill_srv = nh.advertiseService("kill",
                                        &SimFrame::killCallback, this);
        set_sonar_viz_srv = nh.advertiseService("set_sonar_viz",
                                        &SimFrame::setSonarVizCallback, this);
        set_ghost_srv = nh.advertiseService("set_ghost",
                                        &SimFrame::setGhostCallback, this);
        set_teleop_srv = nh.advertiseService("set_teleop",
                                        &SimFrame::setTeleopCallback, this);

        // Subscribe and publisher wirless from robots
        wireless_receive = nh.advertise< ::messages::WirelessPacket>(
            "/wireless/receive", 1000); 
        wireless_send = nh.subscribe("/wireless/send", 1000,
            &SimFrame::wirelessCallback, this);

        // Teleop
        teleop_type = TELEOP_OFF;
        teleop_l_speed = 0;
        teleop_r_speed = 0;
        teleop_scoutname = "scout1";

        teleop_pub = nh.advertise< ::messages::set_motors>("/scout1/set_motors", 1000);

        ROS_INFO("Starting scoutsim with node name %s",
                 ros::this_node::getName().c_str()) ;

        wxMenu *menuFile = new wxMenu;
        menuFile->Append(ID_ABOUT, _("&About"));
        menuFile->AppendSeparator();
        menuFile->Append(ID_QUIT, _("E&xit"));

        wxMenu *menuSim = new wxMenu;
        menuSim->Append(ID_CLEAR, _("&Clear"));

        wxMenu *menuView = new wxMenu;
        menuView->Append(ID_MAP, _("&Map"));
        menuView->Append(ID_LINES, _("&Lines"));
        menuView->Append(ID_WALLS, _("&Walls"));

        wxMenu *menuTeleop = new wxMenu;
        menuTeleop->Append(ID_TELEOP_NONE, _("&None"));
        menuTeleop->Append(ID_TELEOP_PRECISE, _("&Precise"));
        menuTeleop->Append(ID_TELEOP_FLUID, _("&Fluid"));

        wxMenuBar *menuBar = new wxMenuBar;
        menuBar->Append(menuFile, _("&File"));
        menuBar->Append(menuSim, _("&Sim"));
        menuBar->Append(menuView, _("&View"));
        menuBar->Append(menuTeleop, _("&Teleop"));

        SetMenuBar(menuBar);

        width_in_meters = GetSize().GetWidth() / PIX_PER_METER;
        height_in_meters = GetSize().GetHeight() / PIX_PER_METER;

        spawnScout("scout1", 1.4, .78, 0);
    }

    SimFrame::~SimFrame()
    {
        delete update_timer;
    }

    bool SimFrame::spawnCallback(scoutsim::Spawn::Request  &req,
                                 scoutsim::Spawn::Response &res)
    {
        std::string name = spawnScout(req.name, req.x, req.y, req.theta);
        if (name.empty())
        {
            ROS_WARN("A scout named [%s] already exists", req.name.c_str());
            return false;
        }

        res.name = name;
        return true;
    }

    bool SimFrame::killCallback(scoutsim::Kill::Request& req,
                                scoutsim::Kill::Response&)
    {
        M_Scout::iterator it = scouts.find(req.name);
        if (it == scouts.end())
        {
            ROS_WARN("Tried to kill scout [%s], which does not exist",
                     req.name.c_str());
            return false;
        }

        scouts.erase(it);

        return true;
    }

    bool SimFrame::setSonarVizCallback(SetSonarViz::Request& req,
                                       SetSonarViz::Response&)
    {
        M_Scout::iterator it = scouts.find(req.scout_name);
        if (it == scouts.end())
        {
            ROS_WARN("Tried to set sonar on scout [%s], which does not exist",
                     req.scout_name.c_str());
            return false;
        }

        it->second->set_sonar_visual(req.on);
        return true;
    }

    bool SimFrame::setGhostCallback(SetGhost::Request& req,
                                    SetGhost::Response&)
    {
        for (unsigned int i=0; i < ghost_scouts.size(); ++i)
        {
            if (ghost_scouts.at(i)->get_name() == req.scout_name)
            {
                ghost_scouts.at(i)->set_visible(req.on);
                return true;
            }
        }

        ROS_WARN("Tried to set ghost on scout [%s], which does not exist",
                 req.scout_name.c_str());
        return false;
    }

    bool SimFrame::setTeleopCallback(SetTeleop::Request& req,
                                     SetTeleop::Response&)
    {
        std::stringstream ss;
        ss << "/" << req.scout_name << "/set_motors";
        teleop_pub = nh.advertise< ::messages::set_motors>(ss.str(), 1000);
        teleop_scoutname = req.scout_name.c_str();
        ROS_INFO("Teleoping %s...",teleop_scoutname.c_str()); //debug statement

        return true;
    }

    bool SimFrame::hasScout(const std::string& name)
    {
        return scouts.find(name) != scouts.end();
    }

    std::string SimFrame::spawnScout(const std::string& name,
                                     float x, float y, float angle)
    {
        std::string real_name = name;
        if (real_name.empty())
        {
            // Generate the name scoutX, where X is an increasing number.
            do
            {
                std::stringstream ss;
                ss << "scout" << ++id_counter;
                real_name = ss.str();
            }
            while (hasScout(real_name));
        }
        else
        {
            if (hasScout(real_name))
            {
                return "";
            }
        }

        wxImage scout_image;

        // Try to load a name-specific image; if not, load the default scout
        string specific_name = images_path + name + ".png";
        if (fileExists(specific_name))
        {
            scout_image.LoadFile(wxString::FromAscii(specific_name.c_str()));
            scout_image.SetMask(true);
            scout_image.SetMaskColour(255, 255, 255);
        }
        else
        {
            scout_image.LoadFile(
                wxString::FromAscii((images_path + "scout.png").c_str()));
            scout_image.SetMask(true);
            scout_image.SetMaskColour(255, 255, 255);
        }

        ScoutPtr t(new Scout(ros::NodeHandle(real_name),
                   scout_image, Vector2(x, y), &path_bitmap, angle));
        scouts[real_name] = t;

        ghost_scouts.push_back(new GhostScout(ros::NodeHandle(real_name),
                scout_image, Vector2(x, y), &path_bitmap, angle, name));

        ROS_INFO("Spawning scout [%s] at x=[%f], y=[%f], theta=[%f]",
                 real_name.c_str(), x, y, angle);

        return real_name;
    }

    void SimFrame::onQuit(wxCommandEvent& WXUNUSED(event))
    {
        Close(true);
    }

    void SimFrame::onAbout(wxCommandEvent& WXUNUSED(event))
    {
        wxMessageBox(_("Scoutsim is the simulator the Colony Project's scout robot.\n"
                       "\nThe Colony Project is a part of the Carnegie Mellon\n"
                       "Robotics Club. Our goal is to use cooperative low-cost\n"
                       "robots to solve challenging problems."),
                     _("About Scoutsim"),
                     wxOK | wxICON_INFORMATION, this );
    }

    void SimFrame::onClear(wxCommandEvent& WXUNUSED(event))
    {
        clear();
    }

    void SimFrame::showMap(wxCommandEvent& WXUNUSED(event))
    {
        display_map_name = map_base_name;
        clear();
    }

    void SimFrame::showLines(wxCommandEvent& WXUNUSED(event))
    {
        display_map_name = map_lines_name;
        clear();
    }
    
    void SimFrame::showWalls(wxCommandEvent& WXUNUSED(event))
    {
        display_map_name = map_walls_name;
        clear();
    }

    void SimFrame::clear()
    {
        path_dc.SetBackground(wxBrush(wxColour(100, 100, 100)));
        path_dc.Clear();

        sonar_dc.SetBackground(wxBrush(wxColour(255, 0, 0)));
        sonar_dc.Clear();

        sonar_dc.SelectObject(path_bitmap);

        path_bitmap.LoadFile(wxString::FromAscii(display_map_name.c_str()));
        path_dc.SelectObject(path_bitmap);
        SetSize(wxSize(path_bitmap.GetWidth(), path_bitmap.GetHeight()));
    }

    // Runs every REAL_TIME_REFRESH_RATE.
    void SimFrame::onUpdate(wxTimerEvent& evt)
    {
        ros::spinOnce();

        teleop();

        updateScouts();

        if (!ros::ok())
        {
            Close();
        }

        frame_count++;
    }

    void SimFrame::onPaint(wxPaintEvent& evt)
    {
        wxPaintDC dc(this);

        dc.DrawBitmap(path_bitmap, 0, 0, true);

        M_Scout::iterator it = scouts.begin();
        M_Scout::iterator end = scouts.end();
        for (; it != end; ++it)
        {
            it->second->paint(dc);
        }
        for (unsigned int i=0; i < ghost_scouts.size(); ++i)
        {
            ghost_scouts.at(i)->paint(dc);
        }
    }

    bool SimFrame::fileExists(const std::string& filename)
    {
        struct stat buf;
        if (stat(filename.c_str(), &buf) != -1)
        {
            return true;
        }
        return false;
    }

    void SimFrame::stopTeleop(wxCommandEvent& event)
    {
        teleop_type = TELEOP_OFF;
        teleop_l_speed = 0;
        teleop_r_speed = 0;
    }

    void SimFrame::startTeleopPrecise(wxCommandEvent& event)
    {
        teleop_type = TELEOP_PRECISE;
        teleop_l_speed = 0;
        teleop_r_speed = 0;
    }

    void SimFrame::startTeleopFluid(wxCommandEvent& event)
    {
        teleop_type = TELEOP_FLUID;
        teleop_l_speed = 0;
        teleop_r_speed = 0;
        teleop_fluid_speed = 0;
        teleop_fluid_omega = 0;
    }

    void SimFrame::teleop_move_precise()
    {
        // Default to stop
        teleop_l_speed = 0;
        teleop_r_speed = 0;

        if (wxGetKeyState(WXK_UP))
        {
            teleop_l_speed = TELEOP_PRECISE_SPEED;
            teleop_r_speed = TELEOP_PRECISE_SPEED;
        }
        else if (wxGetKeyState(WXK_DOWN))
        {
            teleop_l_speed = -TELEOP_PRECISE_SPEED;
            teleop_r_speed = -TELEOP_PRECISE_SPEED;
        }
        else if (wxGetKeyState(WXK_LEFT))
        {
            teleop_l_speed = -TELEOP_PRECISE_TURN_SPEED;
            teleop_r_speed = TELEOP_PRECISE_TURN_SPEED;
        }
        else if (wxGetKeyState(WXK_RIGHT))
        {
            teleop_l_speed = TELEOP_PRECISE_TURN_SPEED;
            teleop_r_speed = -TELEOP_PRECISE_TURN_SPEED;
        }
    }

    void SimFrame::teleop_move_fluid()
    {
        if (wxGetKeyState(WXK_UP))
        {
            teleop_fluid_speed += TELEOP_FLUID_INC * 2;
        }
        else if (wxGetKeyState(WXK_DOWN))
        {
            teleop_fluid_speed -= TELEOP_FLUID_INC * 2;
        }
        else if (teleop_fluid_speed > TELEOP_FLUID_INC)
        {
            teleop_fluid_speed -= TELEOP_FLUID_INC;
        }
        else if (teleop_fluid_speed < -TELEOP_FLUID_INC)
        {
            teleop_fluid_speed += TELEOP_FLUID_INC;
        }
        else
        {
            teleop_fluid_speed = 0;
        }

        if (wxGetKeyState(WXK_LEFT))
        {
            teleop_fluid_omega -= TELEOP_FLUID_INC * 2;
        }
        else if (wxGetKeyState(WXK_RIGHT))
        {
            teleop_fluid_omega += TELEOP_FLUID_INC * 2;
        }
        else
        {
            teleop_fluid_omega = 0;
        }

        if (teleop_fluid_speed > TELEOP_FLUID_MAX_SPEED)
        {
            teleop_fluid_speed = TELEOP_FLUID_MAX_SPEED;
        }
        else if (teleop_fluid_speed < -TELEOP_FLUID_MAX_SPEED)
        {
            teleop_fluid_speed = -TELEOP_FLUID_MAX_SPEED;
        }
        if (teleop_fluid_omega > TELEOP_FLUID_MAX_SPEED)
        {
            teleop_fluid_omega = TELEOP_FLUID_MAX_SPEED;
        }
        else if (teleop_fluid_omega < -TELEOP_FLUID_MAX_SPEED)
        {
            teleop_fluid_omega = -TELEOP_FLUID_MAX_SPEED;
        }

        int l_speed = teleop_fluid_speed + teleop_fluid_omega;
        int r_speed = teleop_fluid_speed - teleop_fluid_omega;

        teleop_l_speed = max(MIN_ABSOLUTE_SPEED,
                             min(MAX_ABSOLUTE_SPEED, l_speed));
        teleop_r_speed = max(MIN_ABSOLUTE_SPEED,
                             min(MAX_ABSOLUTE_SPEED, r_speed));
    }

    void SimFrame::teleop()
    {
        switch (teleop_type)
        {
            case TELEOP_OFF:
                return;
            case TELEOP_PRECISE:
                teleop_move_precise();
                break;
            case TELEOP_FLUID:
                teleop_move_fluid();
                break;
        }

        ::messages::set_motors msg;
        msg.fl_set = true;
        msg.fr_set = true;
        msg.bl_set = true;
        msg.br_set = true;
        msg.teleop_ON = true;

        msg.fl_speed = teleop_l_speed;
        msg.fr_speed = teleop_r_speed;
        msg.bl_speed = teleop_l_speed;
        msg.br_speed = teleop_r_speed;

        teleop_pub.publish(msg);
    }

    void SimFrame::updateScouts()
    {

        if (last_scout_update.isZero())
        {
            last_scout_update = ros::WallTime::now();
            return;
        }

        path_image = path_bitmap.ConvertToImage();
        Refresh();

        M_Scout::iterator it = scouts.begin();
        M_Scout::iterator end = scouts.end();

        world_state state;
        state.canvas_width = width_in_meters;
        state.canvas_height = height_in_meters;

        for (; it != end; ++it)
        {

            it->second->update(SIM_TIME_REFRESH_RATE,
                               path_dc, sonar_dc,
                               path_image, lines_image, walls_image,
                               path_dc.GetBackground().GetColour(),
                               sonar_dc.GetBackground().GetColour(),
                               state);
        }

        for (unsigned int i = 0; i < ghost_scouts.size(); ++i)
        {
            ghost_scouts.at(i)->update(SIM_TIME_REFRESH_RATE, path_dc, sonar_dc,
                path_dc.GetBackground().GetColour(), state);
        }

        frame_count++;
    }

    bool SimFrame::clearCallback(std_srvs::Empty::Request&,
                                 std_srvs::Empty::Response&)
    {
        ROS_INFO("Clearing scoutsim.");
        clear();
        return true;
    }

    bool SimFrame::resetCallback(std_srvs::Empty::Request&,
                                 std_srvs::Empty::Response&)
    {
        ROS_INFO("Resetting scoutsim.");
        scouts.clear();
        id_counter = 0;
        spawnScout("", width_in_meters / 2.0, height_in_meters / 2.0, 0);
        clear();
        return true;
    }

    void SimFrame::wirelessCallback(const ::messages::WirelessPacket::ConstPtr& msg)
    {
        wireless_receive.publish(msg);
    }
}

/** @} */
