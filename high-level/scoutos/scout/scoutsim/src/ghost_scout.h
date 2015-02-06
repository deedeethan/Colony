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

#ifndef _SCOUTSIM_GHOST_SCOUT_H_
#define _SCOUTSIM_GHOST_SCOUT_H_

#include <ros/ros.h>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <scoutsim/Pose.h>
#include <scoutsim/SetPen.h>
#include <scoutsim/Color.h>

#include "messages/ScoutPosition.h"

#include <geometry_msgs/Pose2D.h>

#include <wx/wx.h>

#include "scout.h"

#include "scoutsim_internal.h"
#include "scout_constants.h"


namespace scoutsim
{
  class GhostScout
  {
    public:
      GhostScout(const ros::NodeHandle& nh,const wxImage& scout_image,
          const Vector2& pos, wxBitmap *path_bitmap, float orient,
          std::string scoutname);

      geometry_msgs::Pose2D update(double dt, wxMemoryDC& path_dc,
          wxMemoryDC& sonar_dc,
          wxColour background_color,
          world_state state);
      void paint(wxDC& dc);

      void setPosition(const ::messages::ScoutPosition& msg);
      std::string get_name();
      void set_visible(bool vis);

    private:
      unsigned int rgb_to_grey(unsigned char r,
          unsigned char g,
          unsigned char b);

      wxBitmap *path_bitmap;

      ros::NodeHandle node;

      wxImage scout_image;
      wxBitmap scout;

      Vector2 pos;
      float start_x, start_y;
      float orient;
      std::string name;

      ros::Subscriber position;

      bool is_visible;
  };
}

#endif
