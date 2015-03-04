#include "Odometry_new.h"
#include "../helper_classes/RungaKutta.h"

using namespace std;

/** Set up the odometry node and prepare communications over ROS */
Odometry_new::Odometry_new(string scoutname, Sensors* sensors):
                Behavior(scoutname, "odometry", sensors)
{
  name = scoutname;
  scout_pos = new pos;

  encoder_readings scout_enc = encoders->query();
  motor_fl_ticks_last = motor_bl_ticks_last = scout_enc.fl_ticks;
  motor_fr_ticks_last = motor_br_ticks_last = scout_enc.fr_ticks;
}


/** Query encoders and integrate using RK4 to estimate position */
void Odometry_new::get_position(double loop_time)
{
  float avg_left_ticks, avg_right_ticks;
  float ang_vl, ang_vr, lin_vl, lin_vr;
  float robot_v, robot_w;

  // figure out the encoder ticks during the time
  encoder_readings scout_enc = encoders->query();
  motor_fl_ticks_iter = scout_enc.fl_ticks - motor_fl_ticks_last;
  motor_fr_ticks_iter = scout_enc.fr_ticks - motor_fr_ticks_last;
  motor_bl_ticks_iter = scout_enc.bl_ticks - motor_bl_ticks_last;
  motor_br_ticks_iter = scout_enc.br_ticks - motor_br_ticks_last;
  
  ROS_INFO("%d  %d", motor_fl_ticks_iter, motor_fr_ticks_iter);

  // update the tick values
  motor_fl_ticks_last = scout_enc.fl_ticks;
  motor_fr_ticks_last = scout_enc.fr_ticks;
  motor_bl_ticks_last = scout_enc.bl_ticks;
  motor_br_ticks_last = scout_enc.br_ticks;

  avg_left_ticks = (float) motor_fl_ticks_iter;
  avg_right_ticks = (float) motor_fr_ticks_iter;
  // right now the back encoders does not work so uncomment the next
  // two lines when the work!!!
  /*
  avg_left_ticks = motor_fl_ticks_iter/(2.0) + motor_bl_ticks_iter/(2.0);
  avg_right_ticks = motor_fr_ticks_iter/(2.0) + motor_br_ticks_iter/(2.0);
  */
  
  // figure out the angular velocity in radians per second
  ang_vl = (avg_left_ticks/loop_time) * RAD_PER_TICK;
  ang_vr = (avg_right_ticks/loop_time) * RAD_PER_TICK;

  // figure out the linear velosity of robot
  lin_vl = ang_vl * WHEEL_R;
  lin_vr = ang_vr * WHEEL_R;
  
  // figure out how the robot speed and turning speed
  robot_v = (lin_vl/2.0 + lin_vr/2.0);
  robot_w = (lin_vr - lin_vl)/WHEEL_B;

  // now use RK4 to integrade the velocities to get the the move in pos
  double state_temp[] = {scout_pos->x, scout_pos->y, scout_pos->theta,
      robot_v, robot_w};
  vector<double> state (state_temp, state_temp+5);

  vector<double> new_state = RungaKutta::rk4(state, RungaKutta::diff_drive_robot, loop_time);

  scout_pos->x = new_state[0];
  scout_pos->y = new_state[1];
  scout_pos->theta = new_state[2];
}

void Odometry_new::run()
{
  scout_position = node.advertise< ::messages::ScoutPosition>(name+"/posn", 1000);
  double last_time = Time::now().toSec();
  double loop_time, current_time;

  // Reset encoders.
  encoders->reset();

  //Rate r(LOOP_RATE); 
  Duration r = Duration(LOOP_TIME);

  motors->set_sides(-50,50, MOTOR_ABSOLUTE);

  while(ok())
  {
    current_time = Time::now().toSec();
    loop_time = current_time - last_time;
    last_time = current_time;

    get_position(loop_time);
    ROS_INFO("loop time %f", loop_time);

    position.name = name;
    position.x = scout_pos->x;
    position.y = scout_pos->y;
    position.theta = scout_pos->theta;
    scout_position.publish(position);

    ROS_INFO("scout is at %f %f theta: %f", position.x, position.y, position.theta);

    r.sleep();
  }
}
