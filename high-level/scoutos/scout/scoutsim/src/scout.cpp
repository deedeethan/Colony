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
 * @file scout.cpp
 *
 * @ingroup scoutsim
 * @{
 */

#include "scout.h"

#include <wx/wx.h>

#define DEFAULT_PEN_R 0xb3
#define DEFAULT_PEN_G 0xb8
#define DEFAULT_PEN_B 0xff

using namespace std;

namespace scoutsim
{
    /**
     * The scout object, which is responsible for refreshing itself and
     * updating its position and simulated sensors.
     *
     * @ingroup scoutsim
     */
    Scout::Scout(const ros::NodeHandle& nh,
                 const wxImage& scout_image,
                 const Vector2& pos,
                 wxBitmap *path_bitmap,
                 float orient)
        : path_bitmap(path_bitmap)
          , sonar_visual_on(false)
          , sonar_on(true)
          , ignore_behavior(false)
          , node (nh)
          , scout_image(scout_image)
          , pos(pos)
          , orient(orient)
          , motor_fl_speed(0)
          , motor_fr_speed(0)
          , motor_bl_speed(0)
          , motor_br_speed(0)
          , fl_ticks(0)
          , fr_ticks(0)
          , bl_ticks(0)
          , br_ticks(0)
          , pen_on(true)
          , pen(wxColour(DEFAULT_PEN_R, DEFAULT_PEN_G, DEFAULT_PEN_B))
    {
        pen.SetWidth(3);
        scout = wxBitmap(scout_image);

        motors_sub = node.subscribe("set_motors", 1, &Scout::setMotors, this);

        pose_pub = node.advertise<Pose>("pose", 1);
        color_pub = node.advertise<Color>("color_sensor", 1);
        sonar_pub = node.advertise< ::messages::sonar_distance>("sonar_distance", 1);
        set_pen_srv = node.advertiseService("set_pen",
                                            &Scout::setPenCallback,
                                            this);
        toggle_sonar_srv = node.advertiseService("sonar_toggle",
                                            &Scout::handle_sonar_toggle,
                                            this);
        set_sonar_srv = node.advertiseService("sonar_set_scan",
                                            &Scout::handle_sonar_set_scan,
                                            this);
        query_encoders_srv =
            node.advertiseService("query_encoders",
                                  &Scout::query_encoders_callback,
                                  this);

        query_linesensor_srv =
            node.advertiseService("query_linesensor",
                                  &Scout::query_linesensor_callback,
                                  this);

        query_BOM_srv =
            node.advertiseService("query_BOM",
                                  &Scout::query_BOM_callback,
                                  this);


        for (unsigned int i = 0; i < NUM_LINESENSORS; i++)
        {
            linesensor_readings.push_back(0);
        }

        for (unsigned int i = 0; i < NUM_BOMS; i++)
        {
            BOM_readings.push_back(0);
            BOM_senders.push_back(0);
        }

        // Initialize sonar
        sonar_position = 0;
        sonar_stop_l = 0;
        sonar_stop_r = 23;
        sonar_direction = 1;
        sonar_tick_time = ros::Duration(scoutsim::SONAR_HALF_SPIN_TIME / 24.0);
        
        // Init latch
        teleop_latch = 0;
    }

    float Scout::absolute_to_mps(int absolute_speed)
    {
        return ((float) absolute_speed) * MAX_SPEED_MPS / MAX_ABSOLUTE_SPEED;
    }

    /**
     * A callback function that sets velocity based on a set_motors
     * request.
     * @todo Use "callback" in all callback function names? Or remove?
     */
    void Scout::setMotors(const ::messages::set_motors::ConstPtr& msg)
    {
        last_command_time = ros::WallTime::now();

        //ignore non-teleop commands if commands if teleop is ON
        //if (node.getNamespace() != current_teleop_scout || msg->teleop_ON)
        //{

        //latch value indicates number of uninterrupted teleop messages
        //before teleop latch shifts again
        if (!(msg->teleop_ON) && teleop_latch < 3)
        {
            teleop_latch++;
        }

        if (!(msg->teleop_ON) || teleop_latch ==0)
        {
            if(msg->fl_set)
            {
                motor_fl_speed = absolute_to_mps(msg->fl_speed);
            }
            if(msg->fr_set)
            {
                motor_fr_speed = absolute_to_mps(msg->fr_speed);
            }
            if(msg->bl_set)
            {
                motor_bl_speed = absolute_to_mps(msg->bl_speed);
            }
            if(msg->br_set)
            {
                motor_br_speed = absolute_to_mps(msg->br_speed);
            }
        }        

        //if a teleop message comes through, decrease the latch
        //latch code works on the assumption there will be more behavior messages
        //than teleop messages
        if (msg->teleop_ON && teleop_latch>0)
        {
            teleop_latch--;
        }
        //}

    }

    bool Scout::handle_sonar_toggle(::messages::sonar_toggle::Request  &req,
                         ::messages::sonar_toggle::Response &res)
    {
        if (req.set_on && !sonar_on)
        {
            ROS_INFO("Turning on the sonar");
            sonar_on = true;

        }
        else if (!req.set_on && sonar_on)
        {
            ROS_INFO("Turning off the sonar");
            sonar_on = false;
        }
        else
        {
            ROS_INFO("Sonar state remains unchanged");
        }
        res.ack = true;
        return true;
    }

    bool Scout::handle_sonar_set_scan(::messages::sonar_set_scan::Request  &req,
                                      ::messages::sonar_set_scan::Response &res)
    {
        // Code to set the sonar to scan from
        // req.stop_l to req.stop_r
        if (req.stop_l>=0 and req.stop_r<=23 and req.stop_l<=req.stop_r)
        {
            ROS_INFO("Setting sonar scan range to [%i, %i]",
                     req.stop_l,
                     req.stop_r);
            sonar_stop_l = req.stop_l;
            sonar_stop_r = req.stop_r;
            sonar_position = req.stop_l;
            sonar_direction = 1;
            res.ack = true;
        }
        else
        {
            ROS_INFO("Bad Input: Input should be integers 0-23, stop_l<stop_r");
        }
        return true;
    }

    bool Scout::setPenCallback(scoutsim::SetPen::Request& req,
                               scoutsim::SetPen::Response&)
    {
        pen_on = !req.off;
        if (req.off)
        {
            return true;
        }

        wxPen pen(wxColour(req.r, req.g, req.b));
        if (req.width != 0)
        {
            pen.SetWidth(req.width);
        }

        pen = pen;
        return true;
    }

    bool Scout::query_encoders_callback(::messages::query_encoders::Request&,
                                        ::messages::query_encoders::Response& res)
    {
        res.fl_distance = fl_ticks;
        res.fr_distance = fr_ticks;
        res.bl_distance = bl_ticks;
        res.br_distance = br_ticks;

        return true;
    }

    bool Scout::query_linesensor_callback(::messages::query_linesensor::Request&,
                                          ::messages::query_linesensor::Response& res)
    {
        res.readings = linesensor_readings;

        return true;
    }

    bool Scout::query_BOM_callback(::messages::query_boms::Request&,
                                   ::messages::query_boms::Response& res)
    {
        res.readings = BOM_readings;
        res.senders = BOM_senders;

        return true;
    }

    // Scale to linesensor value
    unsigned int Scout::rgb_to_grey(unsigned char r,
                                    unsigned char g,
                                    unsigned char b)
    {
        // Should be 0 to 255
        unsigned int grey = ((unsigned int) r + (unsigned int) g + (unsigned int) b) / 3;

        /// @todo Convert to the proper range
        return 255 - grey;
    }

    unsigned int Scout::trace_sonar(const wxImage& walls_image, int x, int y,
                                    double robot_theta, int sonar_pos, 
                                    wxMemoryDC& sonar_dc)
    {
        double angle = robot_theta + (PI * ((float) sonar_pos) / 24.0) - PI / 2;
        unsigned int d = 0;

        unsigned int reading = 0;

        int d_x = 0;
        int d_y = 0;

        do
        {
            d_x = x + (int) floor(d * cos(angle));
            d_y = y - (int) floor(d * sin(angle));

            // Out of image boundary
            if (d_x < 0 || d_x >= walls_image.GetWidth() ||
                d_y < 0 || d_y >= walls_image.GetHeight())
            {
                break;
            }

            // Max range
            if (d > scoutsim::SONAR_MAX_RANGE_PIX)
            {
                break;
            }

            // Get the sonar reading at the current position of the sonar
            unsigned char r = walls_image.GetRed(d_x, d_y);
            unsigned char g = walls_image.GetGreen(d_x, d_y);
            unsigned char b = walls_image.GetBlue(d_x, d_y);
    
            reading = rgb_to_grey(r, g, b);
        
            d++;
        }
        /// @todo Consider using different cutoffs for different features
        while (reading < 128); /// @todo Get rid of hardcoded stuff like this
        
        if (sonar_visual_on)
        {   
            if (isFront)
            {
                // draw a circle at the wall_x, wall_y where reading > 128
                sonar_dc.SelectObject(*path_bitmap);
                sonar_dc.SetBrush(*wxRED_BRUSH); //old value
                sonar_dc.DrawCircle(wxPoint(old_front_dx, old_front_dy), 2);
                old_front_dx = d_x;
                old_front_dy = d_y;
            }
            else
            {
                // draw a circle at the wall_x, wall_y where reading > 128
                sonar_dc.SelectObject(*path_bitmap);
                sonar_dc.SetBrush(*wxRED_BRUSH); //old value
                sonar_dc.DrawCircle(wxPoint(old_back_dx,old_back_dy), 2);
                old_back_dx = d_x;
                old_back_dy = d_y;
            }

            sonar_dc.SetBrush(*wxGREEN_BRUSH);  //newest value
            sonar_dc.DrawCircle(wxPoint(d_x,d_y), 2);
            if (isFront) // @todo for some reason isFront = (!isFront) is not working
            {
                isFront = FALSE;
            }
            else
            {
                isFront = TRUE; 
            }
        }

        // Convert from pixels to mm and return
        return (unsigned int) ((1000 / PIX_PER_METER) * d);
    }

    // x and y is current position of the sonar
    void Scout::update_sonar(const wxImage& walls_image, int x, int y,
                             double robot_theta,wxMemoryDC& sonar_dc)
    {
        // Only rotate the sonar at the correct rate.
        if (ros::Time::now() - last_sonar_time < sonar_tick_time)
        {
            return;
        }
        last_sonar_time = ros::Time::now();

        unsigned int d_front = trace_sonar(walls_image, x, y, robot_theta,
                                           sonar_position, sonar_dc);
        unsigned int d_back = trace_sonar(walls_image, x, y, robot_theta,
                                          sonar_position + 24, sonar_dc);

        // Publish
        ::messages::sonar_distance msg;
        msg.pos = sonar_position;
        msg.distance0 = d_front;
        msg.distance1 = d_back;
        msg.stamp = ros::Time::now();

        sonar_pub.publish(msg);

        // Update the sonar rotation
        if (sonar_position + sonar_direction <= sonar_stop_r &&
            sonar_position + sonar_direction >= sonar_stop_l)
        {
            sonar_position = sonar_position + sonar_direction;
        }
        else
        {
            sonar_direction = -sonar_direction;
        }
    }

    void Scout::update_linesensor(const wxImage& lines_image, int x, int y,
                                  double robot_theta)
    {
        linesensor_readings.clear();

        double spacing = scout_image.GetWidth() / (NUM_LINESENSORS - 1);
        for (int s = 0; s < NUM_LINESENSORS; s++)
        {
            double offset = -(scout_image.GetWidth() / 2) + s * spacing;

            int sensor_x = (int) (x - LNSNSR_D * cos(robot_theta) -
                                  offset * sin(robot_theta));
            int sensor_y = (int) (y + LNSNSR_D * sin(robot_theta) -
                                  offset * cos(robot_theta));

            unsigned char r = lines_image.GetRed(sensor_x, sensor_y);
            unsigned char g = lines_image.GetGreen(sensor_x, sensor_y);
            unsigned char b = lines_image.GetBlue(sensor_x, sensor_y);

            unsigned int reading = rgb_to_grey(r, g, b);

            linesensor_readings.push_back(reading);
        }
    }

    void Scout::update_BOM(int bom_index,
                           unsigned int bom_value,
                           unsigned int sender) {
        BOM_readings[bom_index] = bom_value;
        BOM_senders[bom_index] = sender;
    }

    /// Sends back the position of this scout so scoutsim can save
    /// the world state
    /// @todo remove dt param
    geometry_msgs::Pose2D Scout::update(double dt, wxMemoryDC& path_dc,
                                        wxMemoryDC& sonar_dc,
                                        const wxImage& path_image,
                                        const wxImage& lines_image,
                                        const wxImage& walls_image,
                                        wxColour background_color,
                                        wxColour sonar_color,
                                        world_state state)
    {
        // Assume that the two motors on the same side will be set to
        // roughly the same speed. Does not account for slip conditions
        // when they are set to different speeds.
        float l_speed = (float (motor_fl_speed + motor_bl_speed)) / 2;
        float r_speed = (float (motor_fr_speed + motor_br_speed)) / 2;

        // Find linear and angular movement in m
        float lin_dist = SIM_TIME_REFRESH_RATE * (l_speed + r_speed) / 2;
        float ang_dist = SIM_TIME_REFRESH_RATE * (r_speed - l_speed) / SCOUT_WIDTH;

        //store currently teleop'd scoutname
        //std::stringstream ss;
        //ss << "/" << teleop_scoutname;
        //current_teleop_scout = ss.str();

        Vector2 old_pos = pos;

        // Update encoders
        fl_ticks += (unsigned int) (motor_fl_speed * SIM_TIME_REFRESH_RATE *
                                    ENCODER_TICKS_PER_METER);
        fr_ticks += (unsigned int) (motor_fr_speed * SIM_TIME_REFRESH_RATE *
                                    ENCODER_TICKS_PER_METER);
        bl_ticks += (unsigned int) (motor_bl_speed * SIM_TIME_REFRESH_RATE *
                                    ENCODER_TICKS_PER_METER);
        br_ticks += (unsigned int) (motor_br_speed * SIM_TIME_REFRESH_RATE *
                                    ENCODER_TICKS_PER_METER);

        orient = fmod((float) (orient + ang_dist), (float) (2.0 * PI));

        pos.x += cos(orient) * lin_dist;
        pos.y -= sin(orient) * lin_dist; //Subtract because of the way the simulator handles y directions.

        // Clamp to screen size
        if (pos.x < 0 || pos.x >= state.canvas_width
                || pos.y < 0 || pos.y >= state.canvas_height)
        {
            ROS_WARN("I hit the wall! (Clamping from [x=%f, y=%f])", pos.x, pos.y);
        }

        pos.x = min(max(pos.x, 0.0f), state.canvas_width);
        pos.y = min(max(pos.y, 0.0f), state.canvas_height);

        int canvas_x = pos.x * PIX_PER_METER;
        int canvas_y = pos.y * PIX_PER_METER;


        {
            wxImage rotated_image =
                scout_image.Rotate(orient - PI/2.0,
                                   wxPoint(scout_image.GetWidth() / 2,
                                           scout_image.GetHeight() / 2),
                                   false);

            for (int y = 0; y < rotated_image.GetHeight(); ++y)
            {
                for (int x = 0; x < rotated_image.GetWidth(); ++x)
                {
                    if (rotated_image.GetRed(x, y) == 255
                            && rotated_image.GetBlue(x, y) == 255
                            && rotated_image.GetGreen(x, y) == 255)
                    {
                        rotated_image.SetAlpha(x, y, 0);
                    }
                }
            }

            scout = wxBitmap(rotated_image);
        }

        Pose p;
        p.x = pos.x;
        p.y = pos.y;
        p.theta = orient;
        p.linear_velocity = l_speed;
        p.angular_velocity = r_speed;
        pose_pub.publish(p);

        update_linesensor(lines_image, canvas_x, canvas_y, p.theta);
        if (sonar_on)
        {
            update_sonar(walls_image,
                         canvas_x + scoutsim::SCOUT_SONAR_X,
                         canvas_y + scoutsim::SCOUT_SONAR_Y,
                         p.theta,sonar_dc);

        }

        // Figure out (and publish) the color underneath the scout
        {
            //wxSize scout_size = wxSize(scout.GetWidth(), scout.GetHeight());
            Color color;
            color.r = path_image.GetRed(canvas_x, canvas_y);
            color.g = path_image.GetGreen(canvas_x, canvas_y);
            color.b = path_image.GetBlue(canvas_x, canvas_y);
            color_pub.publish(color);
        }

        ROS_DEBUG("[%s]: pos_x: %f pos_y: %f theta: %f",
                  node.getNamespace().c_str(), pos.x, pos.y, orient);

        if (pen_on)
        {
            if (pos != old_pos)
            {
                path_dc.SelectObject(*path_bitmap);
                path_dc.SetPen(pen);
                path_dc.DrawLine(pos.x * PIX_PER_METER, pos.y * PIX_PER_METER,
                                 old_pos.x * PIX_PER_METER, old_pos.y * PIX_PER_METER);
            }
        }

        geometry_msgs::Pose2D my_pose;
        my_pose.x = pos.x;
        my_pose.y = pos.y;
        my_pose.theta = orient;

        return my_pose;
    }

    void Scout::paint(wxDC& dc)
    {
        wxSize scout_size = wxSize(scout.GetWidth(), scout.GetHeight());
        dc.DrawBitmap(scout, pos.x * PIX_PER_METER - (scout_size.GetWidth() / 2),
                      pos.y * PIX_PER_METER - (scout_size.GetHeight() / 2), true);
    }

    void Scout::set_sonar_visual(bool on)
    {
        sonar_visual_on = on;
    }
}

/** @} */
