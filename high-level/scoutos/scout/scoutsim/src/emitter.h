#ifndef _SCOUTSIM_EMITTER_H_
#define _SCOUTSIM_EMITTER_H_

#include <ros/ros.h>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <scoutsim/Pose.h>
#include <scoutsim/SetPen.h>
#include <scoutsim/Color.h>

#include <geometry_msgs/Pose2D.h>

#include <wx/wx.h>

#include "scout.h"
#include "scoutsim_internal.h"
#include "scout_constants.h"

#define PI 3.14159265

#define NUM_LINESENSORS 8

// Distance, pixels, from center of robot to the linesensors.
#define LNSNSR_D 20

#define BOM_EM_APERTURE (PI * 0.05) // 9 degrees
#define BOM_EM_DISTANCE 5.0
#define BOM_REC_APERTURE (PI / 6) // 30 degrees
#define BOM_REC_DISTANCE 5.0

namespace scoutsim
{

    class Emitter
    {
        public:
            Emitter(const ros::NodeHandle& nh,const wxImage& emitter_image,
                  const Vector2& pos, wxBitmap *path_bitmap, float orient,
                  float aperture, int distance);

            geometry_msgs::Pose2D update(double dt, wxMemoryDC& path_dc,
                                         const wxImage& path_image,
                                         const wxImage& lines_image,
                                         const wxImage& walls_image,
                                         wxColour background_color,
                                         world_state state);

            geometry_msgs::Pose2D get_pos();
            void paint(wxDC& dc);
            void set_emitter_visual(bool on);

        private:
            float absolute_to_mps(int absolute_speed);
            bool setPenCallback(scoutsim::SetPen::Request&,
                                scoutsim::SetPen::Response&);
            void setMotors(const messages::set_motors::ConstPtr& msg);
            unsigned int rgb_to_grey(unsigned char r,
                                     unsigned char g,
                                     unsigned char b);
            bool isFront;
            
            int teleop_latch;

	        wxBitmap *path_bitmap;
	        bool ignore_behavior;
            
            //std::string current_teleop_scout;

            ros::NodeHandle node;

            wxImage emitter_image;
            wxBitmap emitter;

            Vector2 pos;
            float orient;

            // Each emitter has a unique id number, which is also displayed on its image.
            int emitter_id;

            float aperture;
            int distance;

            bool pen_on;
            bool emitter_visual_on;
            wxPen pen;

            ros::Publisher color_pub;
            ros::ServiceServer set_pen_srv;

            ros::WallTime last_command_time;
    };
    typedef boost::shared_ptr<Emitter> EmitterPtr;
}

#endif
