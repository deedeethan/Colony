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
 * @file ghost_scout.cpp
 *
 * @ingroup scoutsim
 * @{
 */

#include "ghost_scout.h"

#include <wx/wx.h>

using namespace std;

namespace scoutsim
{
  GhostScout::GhostScout(const ros::NodeHandle& nh,
      const wxImage& scout_image,
      const Vector2& pos,
      wxBitmap *path_bitmap,
      float orient,
      string scoutname)
    : path_bitmap(path_bitmap)
      , node (nh)
      , scout_image(scout_image)
      , pos(pos)
      , start_x(pos.x)
      , start_y(pos.y)
      , orient(orient)
      , name(scoutname)
      , is_visible(false)
  {
    scout = wxBitmap(scout_image);

    position = node.subscribe("position", 1, &GhostScout::setPosition, this);

  }

  void GhostScout::setPosition(const ::messages::ScoutPosition& msg)
  {
    pos.x = start_x + msg.x;
    pos.y = start_y - msg.y;
    orient = msg.theta;
  }

  // Scale to linesensor value
  unsigned int GhostScout::rgb_to_grey(unsigned char r,
      unsigned char g,
      unsigned char b)
  {
    // Should be 0 to 255
    unsigned int grey = ((unsigned int) r + (unsigned int) g + (unsigned int) b) / 3;

    /// @todo Convert to the proper range
    return 255 - grey;
  }

  /// Sends back the position of this scout so scoutsim can save
  /// the world state
  geometry_msgs::Pose2D GhostScout::update(double dt,
      wxMemoryDC& path_dc,
      wxMemoryDC& sonar_dc,
      wxColour background_color,
      world_state state)
  {

    wxImage rotated_image = scout_image.Rotate(orient - M_PI/2.0,
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
        else
        {
          rotated_image.SetAlpha(x, y, 125);
        }
      }
    }

    scout = wxBitmap(rotated_image);

    geometry_msgs::Pose2D my_pose;
    my_pose.x = pos.x;
    my_pose.y = pos.y;
    my_pose.theta = orient;

    return my_pose;
  }

  void GhostScout::paint(wxDC& dc)
  {
      if (!is_visible)
      {
          return;
      }

    wxSize scout_size = wxSize(scout.GetWidth(), scout.GetHeight());
    dc.DrawBitmap(scout, pos.x * PIX_PER_METER - (scout_size.GetWidth() / 2),
        pos.y * PIX_PER_METER - (scout_size.GetHeight() / 2), true);
  }

  string GhostScout::get_name()
  {
    return name;
  }

  void GhostScout::set_visible(bool vis)
  {
    is_visible = vis;
  }
}

/** @} */
