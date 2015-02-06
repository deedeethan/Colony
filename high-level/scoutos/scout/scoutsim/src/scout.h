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

#ifndef _SCOUTSIM_SCOUT_H_
#define _SCOUTSIM_SCOUT_H_

#include <ros/ros.h>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <messages/set_motors.h>
#include <messages/query_encoders.h>
#include <messages/query_linesensor.h>
#include <messages/query_boms.h>
#include <messages/sonar_distance.h>
#include <messages/sonar_toggle.h>
#include <messages/sonar_set_scan.h>

#include <scoutsim/Pose.h>
#include <scoutsim/SetPen.h>
#include <scoutsim/Color.h>

#include <geometry_msgs/Pose2D.h>

#include <wx/wx.h>

#include "scoutsim_internal.h"
#include "scout_constants.h"

#define PI 3.14159265

#define NUM_LINESENSORS 8
#define NUM_BOMS 10

// Distance, pixels, from center of robot to the linesensors.
#define LNSNSR_D 20

namespace scoutsim
{
    struct Vector2
    {
        Vector2()
            : x(0.0)
              , y(0.0)
        {}

        Vector2(float new_x, float new_y)
            : x(new_x)
              , y(new_y)
        {}

        bool operator==(const Vector2& rhs)
        {
            return x == rhs.x && y == rhs.y;
        }

        bool operator!=(const Vector2& rhs)
        {
            return x != rhs.x || y != rhs.y;
        }

        float x;
        float y;
    };

    class Scout
    {
        public:
            Scout(const ros::NodeHandle& nh,const wxImage& scout_image,
                  const Vector2& pos, wxBitmap *path_bitmap, float orient);

            geometry_msgs::Pose2D update(double dt, wxMemoryDC& path_dc,
                                         wxMemoryDC& sonar_dc,
                                         const wxImage& path_image,
                                         const wxImage& lines_image,
                                         const wxImage& walls_image,
                                         wxColour background_color,
                                         wxColour sonar_color,
                                         world_state state);
            void paint(wxDC& dc);
            void set_sonar_visual(bool on);
            void update_BOM(int bom_index, unsigned int bom_value,
                            unsigned int sender);

        private:
            float absolute_to_mps(int absolute_speed);
            void setMotors(const messages::set_motors::ConstPtr& msg);
            bool setPenCallback(scoutsim::SetPen::Request&,
                                scoutsim::SetPen::Response&);
            bool query_encoders_callback(messages::query_encoders::Request&,
                                     messages::query_encoders::Response&);
            bool query_linesensor_callback(messages::query_linesensor::Request&,
                                     messages::query_linesensor::Response&);
            bool query_BOM_callback(messages::query_boms::Request&,
                                     messages::query_boms::Response&);
            bool handle_sonar_toggle(messages::sonar_toggle::Request  &req,
                                     messages::sonar_toggle::Response &res);
            bool handle_sonar_set_scan(messages::sonar_set_scan::Request  &req,
                                     messages::sonar_set_scan::Response &res);
            unsigned int rgb_to_grey(unsigned char r,
                                     unsigned char g,
                                     unsigned char b);
            unsigned int trace_sonar(const wxImage& walls_image, int x, int y,
                                     double robot_theta, int sonar_pos,
                                     wxMemoryDC& sonar_dc);

            void update_sonar(const wxImage& walls_image, int x, int y,
                              double robot_theta, wxMemoryDC& sonar_dc);
            void update_linesensor(const wxImage& lines_image, int x, int y,
                                   double theta);

            int old_front_dx;
            int old_front_dy;
            int old_back_dx;
            int old_back_dy;
            bool isFront;
            
            int teleop_latch;

	        wxBitmap *path_bitmap;
	        bool sonar_visual_on;
            bool sonar_on;
            bool ignore_behavior;
            
            //std::string current_teleop_scout;

            ros::NodeHandle node;

            wxImage scout_image;
            wxBitmap scout;

            Vector2 pos;
            float orient;

            /// @todo should these be an array or something?

            // Keep track of the last commanded speeds sent to the sim,
            // converted to m/s
            float motor_fl_speed;
            float motor_fr_speed;
            float motor_bl_speed;
            float motor_br_speed;

            // Keep track of encoder ticks for each motor
            unsigned int fl_ticks;
            unsigned int fr_ticks;
            unsigned int bl_ticks;
            unsigned int br_ticks;

            int sonar_position;
            int sonar_stop_l;
            int sonar_stop_r;
            int sonar_direction;

            // The last time the sonar changed its position.
            ros::Time last_sonar_time;
            ros::Duration sonar_tick_time;

            // A vector of the 8 linesensor readings
            std::vector<unsigned int> linesensor_readings;
            std::vector<unsigned int> BOM_readings;
            std::vector<unsigned int> BOM_senders;

            // Each scout has a unique id number, which is also displayed on its image.
            int scout_id;

            bool pen_on;
            wxPen pen;

            ros::Subscriber motors_sub;
            ros::Publisher pose_pub;
            ros::Publisher color_pub;
            ros::Publisher sonar_pub;
            ros::Publisher bom_pub;
            ros::ServiceServer set_pen_srv;
            ros::ServiceServer query_encoders_srv;
            ros::ServiceServer query_linesensor_srv;
            ros::ServiceServer query_BOM_srv;
            ros::ServiceServer toggle_sonar_srv;
            ros::ServiceServer set_sonar_srv;

            ros::WallTime last_command_time;
    };
    typedef boost::shared_ptr<Scout> ScoutPtr;
}

#endif
