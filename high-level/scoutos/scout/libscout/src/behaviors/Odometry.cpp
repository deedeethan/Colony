
#include "Odometry.h"

using namespace std;

/** Set up the odometry node and prepare communcations over ROS */
Odometry::Odometry(string scoutname, Sensors* sensors)
    : Behavior(scoutname, "odometry", sensors)
{
    name = scoutname;
    scout_pos = new pos;
    motor_fl_ticks = motor_fr_ticks = motor_bl_ticks = motor_br_ticks = 0;
}

/** Set up the odometry node and prepare communcations over ROS */
    Odometry::Odometry(string scoutname, string behavior_name, Sensors* sensors)
: Behavior(scoutname, behavior_name, sensors)
{
    name = scoutname;
    scout_pos = new pos;
    motor_fl_ticks = motor_fr_ticks = motor_bl_ticks = motor_br_ticks = 0;
}

void Odometry::wait(float duration)
{
    ros::Rate r(WAIT_HZ);
    int ticks = int(duration * WAIT_HZ);
    for (int i = 0; i < ticks; i++)
    {
        get_position();
        spinOnce();
        r.sleep();
    }
}

/** Query encoders and estimate position based on encoder reading */
void Odometry::get_position()
{
    float left_dist, right_dist, total_dist, theta;

    encoder_readings scout_enc = encoders->query();
    motor_fl_dist = scout_enc.fl_ticks - motor_fl_ticks;
    motor_fr_dist = scout_enc.fr_ticks - motor_fr_ticks;
    motor_bl_dist = scout_enc.bl_ticks - motor_bl_ticks;
    motor_br_dist = scout_enc.br_ticks - motor_br_ticks;

    // Get Left and Right distance
    left_dist = DIST_PER_TICK*(((float)(motor_fl_dist))/2.0 +
            ((float)(motor_bl_dist))/2.0);
    right_dist = DIST_PER_TICK*(((float)(motor_fr_dist))/2.0 + 
            ((float)(motor_br_dist))/2.0);

    total_dist = (left_dist)/2.0 + (right_dist)/2.0;
    theta = scout_pos->theta + atan2(right_dist - left_dist, WHEEL_BASE);

    //Use negative theta because we measure theta from 0 on the
    //right to 180 on the left counter-clock-wise, but this is
    //negative theta in the coordinate frame.
    //Also, subtract the delta from y because positive y is down.
    scout_pos->x += total_dist*cos(-theta);
    scout_pos->y -= total_dist*sin(-theta);
    scout_pos->theta = fmod(theta, (float)(2*M_PI));

    //Save state for next time in.
    motor_fl_ticks = scout_enc.fl_ticks;
    motor_fr_ticks = scout_enc.fr_ticks;
    motor_bl_ticks = scout_enc.bl_ticks;
    motor_br_ticks = scout_enc.br_ticks;


    return;
}

void Odometry::run()
{
    scout_position = node.advertise< ::messages::ScoutPosition>(name+"/position", 1000); 

    while(ok())
    {
        get_position();

        position.name = name;
        position.x = scout_pos->x;
        position.y = scout_pos->y;
        position.theta = scout_pos->theta;
        scout_position.publish(position);
    }
}
