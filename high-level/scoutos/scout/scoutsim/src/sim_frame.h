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

#include <wx/wx.h>
#include <wx/event.h>
#include <wx/timer.h>
#include <wx/string.h>
#include <wx/utils.h>

#include <sys/stat.h>

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <scoutsim/Spawn.h>
#include <scoutsim/Kill.h>
#include <scoutsim/SetSonarViz.h>
#include <scoutsim/SetGhost.h>
#include <scoutsim/SetTeleop.h>
#include <messages/set_motors.h>
#include <map>

#include "scout.h"
#include "emitter.h"
#include "ghost_scout.h"
#include "scoutsim_internal.h"
#include "messages/WirelessPacket.h"

#define SCOUTSIM_NUM_SCOUTS 1
#define ID_ABOUT 1
#define ID_QUIT 2
#define ID_CLEAR 3
#define ID_MAP 4
#define ID_LINES 5
#define ID_WALLS 6
#define ID_TELEOP_NONE 7
#define ID_TELEOP_PRECISE 8
#define ID_TELEOP_FLUID 9

// Absolute speeds (-100 - 100)
/// @todo: Clean this up a little; we should be risking overflowing shorts.
#define TELEOP_PRECISE_SPEED 47
#define TELEOP_PRECISE_TURN_SPEED 98
#define TELEOP_FLUID_MAX_SPEED 79
#define TELEOP_FLUID_INC 6

// Teleop types
#define TELEOP_OFF 0
#define TELEOP_PRECISE 1
#define TELEOP_FLUID 2

//Scout DImensions
#define SCOUT_H 0.25
#define SCOUT_W 0.125

namespace scoutsim
{
    class SimFrame : public wxFrame
    {
        public:
            SimFrame(wxWindow* parent, std::string map_name);
            ~SimFrame();

            std::string spawnScout(const std::string& name,
                    float x, float y, float angle);
            std::string spawnEmitter(const std::string& name,
                    float x, float y, float angle);

            void onQuit(wxCommandEvent& event);
            void onAbout(wxCommandEvent& event);
            void onClear(wxCommandEvent& event);
            void showMap(wxCommandEvent& event);
            void showLines(wxCommandEvent& event);
            void showWalls(wxCommandEvent& event);
            void stopTeleop(wxCommandEvent& event);
            void startTeleopPrecise(wxCommandEvent& event);
            void startTeleopFluid(wxCommandEvent& event);

            DECLARE_EVENT_TABLE()

        private:

            typedef std::map<std::string, ScoutPtr> M_Scout;
            typedef std::map<std::string, EmitterPtr> M_Emitter;

            void onUpdate(wxTimerEvent& evt);
            void onPaint(wxPaintEvent& evt);

            bool fileExists(const std::string& filename);

            void teleop_move_precise();
            void teleop_move_fluid();
            void teleop();

            void updateScouts();
            void clear();
            bool hasScout(const std::string& name);
            bool hasEmitter(const std::string& name);

            bool clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
            bool resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
            bool spawnCallback(Spawn::Request&, Spawn::Response&);
            bool spawnEmCallback(Spawn::Request&, Spawn::Response&);
            bool killCallback(Kill::Request&, Kill::Response&);

            bool setSonarVizCallback(SetSonarViz::Request&, SetSonarViz::Response&);
            bool setGhostCallback(SetGhost::Request&, SetGhost::Response&);
            bool setTeleopCallback(SetTeleop::Request&, SetTeleop::Response&);

            void wirelessCallback(const messages::WirelessPacket::ConstPtr& msg);
            bool is_inrange(geometry_msgs::Pose2D scout_pos, 
                         geometry_msgs::Pose2D emitter_pos,
                         double aperture, double distance);
            void checkBOM(geometry_msgs::Pose2D scout_pos,
                          M_Scout::iterator it); 
            int getActiveEmitter(geometry_msgs::Pose2D bom_pos);
            

            ros::Subscriber wireless_send;
            ros::Publisher wireless_receive;

            // Teleop
            short teleop_l_speed;
            short teleop_r_speed;
            ros::Publisher teleop_pub;
            std::string teleop_scoutname;
            int teleop_type;

            int teleop_fluid_speed;
            int teleop_fluid_omega;

            ros::NodeHandle nh;
            wxTimer* update_timer;
            wxBitmap path_bitmap;
            wxImage path_image;
            wxImage lines_image;
            wxImage walls_image;
            wxMemoryDC path_dc;
            wxMemoryDC sonar_dc;

            uint64_t frame_count;

            ros::WallTime last_scout_update;

            ros::ServiceServer clear_srv;
            ros::ServiceServer reset_srv;
            ros::ServiceServer spawn_srv;
            ros::ServiceServer spawn_em_srv;
            ros::ServiceServer kill_srv;
            ros::ServiceServer set_sonar_viz_srv;
            ros::ServiceServer set_ghost_srv;
            ros::ServiceServer set_teleop_srv;

            M_Scout scouts;
            M_Emitter emitters;
            std::vector<GhostScout*> ghost_scouts;
            uint32_t id_counter;
            uint32_t BOM_counter;

            std::string images_path;

            float width_in_meters;
            float height_in_meters;

            //Emitter default values
            double em_aperture;
            double em_distance;


            std::string map_base_name;
            std::string map_lines_name;
            std::string map_walls_name;
            std::string display_map_name;
    };

}
