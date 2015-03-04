// a smarter Odometry the use RK4 integration

#ifndef _ODOMETRY_NEW_H_
#define _ODOMETRY_NEW_H_

#include "../Behavior.h"
#include "../Sensors.h"
#include "messages/ScoutPosition.h"
#include "../helper_classes/ScoutPosition.h"

#define WHEEL_R 0.025 //this is a virtual unit now... use real values
#define WHEEL_B 0.115  // this is a virtual unit now... use real values
#define TICKS_PER_REV 48
#define RAD_PER_TICK (2*M_PI/TICKS_PER_REV)
#define LOOP_RATE 10 // Hz, integrate 10 times per second
#define LOOP_TIME (1.0/LOOP_RATE) // secs

class Odometry_new : Behavior{

  public:

    /** Set up the odometry node and prepare communications over ROS */
    Odometry_new(std::string scoutname, Sensors* sensors);

    /** Query encoders and estimate position based on encoder reading */
    void get_position(double loop_time);

    /** Gets scout position and prints to screen */
    void run();
    
  private:

    /** ROS publisher and client declaration */
    ros::NodeHandle node;
    ros::Publisher scout_position;
    ros::ServiceClient query_encoders_client;
    messages::ScoutPosition position;

    std::string name;

    // FIXME: not sure whether these would overflow....
    int motor_fl_ticks_iter;
    int motor_fr_ticks_iter;
    int motor_bl_ticks_iter;
    int motor_br_ticks_iter;
    
    int motor_fl_ticks_last;
    int motor_fr_ticks_last;
    int motor_bl_ticks_last;
    int motor_br_ticks_last;
    int scout_theta;

    pos* scout_pos;

};

#endif
