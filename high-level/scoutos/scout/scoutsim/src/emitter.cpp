/**
 BETA version of Emitter object for use in BOM simulator
 */

#include "emitter.h"

#include <wx/wx.h>

#define DEFAULT_PEN_R 0xb3
#define DEFAULT_PEN_G 0xb8
#define DEFAULT_PEN_B 0xff

using namespace std;

namespace scoutsim
{
    /**
     * The emitter object, which is responsible for refreshing itself and
     * updating its position.
     *
     */
    Emitter::Emitter(const ros::NodeHandle& nh,
                 const wxImage& emitter_image,
                 const Vector2& pos,
                 wxBitmap *path_bitmap,
                 float orient,
                 float aperture, 
                 int distance)
        : path_bitmap(path_bitmap)
          , ignore_behavior(false)
          , node (nh)
          , emitter_image(emitter_image)
          , pos(pos)
          , orient(orient)
          , aperture(aperture)
          , distance(distance)
          , pen_on(true)
          , emitter_visual_on(true)
          , pen(wxColour(DEFAULT_PEN_R, DEFAULT_PEN_G, DEFAULT_PEN_B))
    {
        pen.SetWidth(3);
        emitter = wxBitmap(emitter_image);
        color_pub = node.advertise<Color>("color_sensor", 1);
        set_pen_srv = node.advertiseService("set_pen",
                                            &Emitter::setPenCallback,
                                            this);
        // Init latch
        teleop_latch = 0;
    }

   
    bool Emitter::setPenCallback(scoutsim::SetPen::Request& req,
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

    // Scale to linesensor value
    unsigned int Emitter::rgb_to_grey(unsigned char r,
                                    unsigned char g,
                                    unsigned char b)
    {
        // Should be 0 to 255
        unsigned int grey = ((unsigned int) r + (unsigned int) g + (unsigned int) b) / 3;

        /// @todo Convert to the proper range
        return 255 - grey;
    }

    /// Sends back the position of this emitter so scoutsim can save
    /// the world state
    /// @todo remove dt param
    geometry_msgs::Pose2D Emitter::update(double dt, wxMemoryDC& path_dc,
                                        const wxImage& path_image,
                                        const wxImage& lines_image,
                                        const wxImage& walls_image,
                                        wxColour background_color,
                                        world_state state)
    {

        pos.x = min(max(pos.x, 0.0f), state.canvas_width);
        pos.y = min(max(pos.y, 0.0f), state.canvas_height);

        //int canvas_x = pos.x * PIX_PER_METER;
        //int canvas_y = pos.y * PIX_PER_METER;


        {
            wxImage rotated_image =
                emitter_image.Rotate(orient - PI/2.0,
                                   wxPoint(emitter_image.GetWidth() / 2,
                                           emitter_image.GetHeight() / 2),
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

            emitter = wxBitmap(rotated_image);
        }

        if (emitter_visual_on) {
            path_dc.SelectObject(*path_bitmap);
            path_dc.SetBrush(*wxGREEN_BRUSH);
            path_dc.DrawLine(
                pos.x * PIX_PER_METER,
                pos.y * PIX_PER_METER,
                (pos.x+cos(orient-BOM_EM_APERTURE/2)*
                    BOM_EM_DISTANCE)*PIX_PER_METER,
                (pos.y-sin(orient-BOM_EM_APERTURE/2)*
                    BOM_EM_DISTANCE)*PIX_PER_METER);
            path_dc.DrawLine(
                pos.x * PIX_PER_METER,
                pos.y * PIX_PER_METER,
                (pos.x+cos(orient+BOM_EM_APERTURE/2)*
                    BOM_EM_DISTANCE)*PIX_PER_METER,
                (pos.y-sin(orient+BOM_EM_APERTURE/2)*
                    BOM_EM_DISTANCE)*PIX_PER_METER);


            path_dc.DrawCircle(
                wxPoint((pos.x+cos(orient-BOM_EM_APERTURE/2)*
                            BOM_EM_DISTANCE)*PIX_PER_METER,
                        (pos.y-sin(orient-BOM_EM_APERTURE/2)*
                            BOM_EM_DISTANCE)*PIX_PER_METER)
                ,2);
            path_dc.DrawCircle(
                wxPoint((pos.x+cos(orient+BOM_EM_APERTURE/2)*
                            BOM_EM_DISTANCE)*PIX_PER_METER,
                        (pos.y-sin(orient+BOM_EM_APERTURE/2)*
                            BOM_EM_DISTANCE)*PIX_PER_METER)
              ,2);
        }

        geometry_msgs::Pose2D my_pose;
        my_pose.x = pos.x;
        my_pose.y = pos.y;
        my_pose.theta = orient;

        return my_pose;
    }

    geometry_msgs::Pose2D Emitter::get_pos()
    {

        geometry_msgs::Pose2D my_pose;
        my_pose.x = pos.x;
        my_pose.y = pos.y;
        my_pose.theta = orient;

        return my_pose;
    }

    void Emitter::paint(wxDC& dc)
    {
        wxSize emitter_size = wxSize(emitter.GetWidth(), emitter.GetHeight());
        dc.DrawBitmap(emitter, pos.x * PIX_PER_METER - (emitter_size.GetWidth() / 2),
                      pos.y * PIX_PER_METER - (emitter_size.GetHeight() / 2), true);
    }

    void Emitter::set_emitter_visual(bool on)
    {
        emitter_visual_on = on;
    }
}

/** @} */
