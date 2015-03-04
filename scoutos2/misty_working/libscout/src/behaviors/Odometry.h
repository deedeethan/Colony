#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include "../Behavior.h"
#include "../Sensors.h"
#include "messages/ScoutPosition.h"
#include "../helper_classes/ScoutPosition.h"

#define WHEEL_RADIUS  .2
#define WHEEL_CIRCUM  (2*M_PI*WHEEL_RADIUS)
#define WHEEL_BASE    1.2
#define ENCODER_COUNT (2*WHEEL_RADIUS*350*M_PI/WHEEL_BASE)
#define DIST_PER_TICK (WHEEL_CIRCUM/ENCODER_COUNT)

class Odometry : public Behavior {

    public:

        /** Set up the odometry node and prepare communcations over ROS */
        Odometry(std::string scoutname, Sensors* sensors);

        /** Set up the odometry node and prepare communcations over ROS */
        Odometry(std::string scoutname, std::string behavior_name, Sensors* sensors);

        /** Query encoders and estimate position based on encoder reading */
        void get_position();

        /** Gets scout position and prints to screen */
        void run();

    protected:

        virtual void wait(float duration);
        pos* scout_pos;

    private:

        /** ROS publisher and client declaration */
        ros::NodeHandle node;
        ros::Publisher scout_position;
        ros::ServiceClient query_encoders_client;
        messages::ScoutPosition position;

        std::string name;

        float msg_time_in;

        int motor_fl_dist;
        int motor_fr_dist;
        int motor_bl_dist;
        int motor_br_dist;

        unsigned int motor_fl_ticks;
        unsigned int motor_fr_ticks;
        unsigned int motor_bl_ticks;
        unsigned int motor_br_ticks;
        float scout_theta;
};

#endif
