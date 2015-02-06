#include "ros.h"
#include "bom.h"
#include "range.h"
#include "stepper.h"
#include "orb.h"
#include "cliffSensor.h"
#include <messages/bom.h>
#include <messages/sonar_distance.h>
#include <messages/sonar_toggle.h>
#include <messages/sonar_set_scan.h>
#include <messages/set_headlights.h>
#include <messages/cliff_status_changed.h>
#include <util/delay.h>

/* Period of main loop in ms */
#define MAINLOOP_PERIOD 100//50

char range_enabled = 0;

void orb_callback(const messages::set_headlights& msg)
{
  orb_set0(msg.left_red, msg.left_green, msg.left_blue);
  orb_set1(msg.right_red, msg.right_green, msg.right_blue);
}

/* dammit, Priya, this capitalization just looks ridiculous */
void range_toggle_cb(const messages::sonar_toggleRequest& req,
    messages::sonar_toggleResponse& resp)
{
  range_enabled = req.set_on;
  if (range_enabled)
    step_enable();
  else
    step_disable();
  resp.ack = true;
}

void range_set_scan_cb(const messages::sonar_set_scanRequest& req,
    messages::sonar_set_scanResponse& resp)
{
  step_sweep_bounds(req.stop_l, req.stop_r);
  resp.ack = true;
}

int main()
{
  unsigned long now, next;
  unsigned int ranges[2];
  char i, id, cliff_status;

  ros::NodeHandle nh;
  nh.initNode();

  /* Stepper */
  // TODO ROS messages to set bounds
  step_init();
  step_dir(1);
  step_set_size(STEP_HALF);
  step_sweep_bounds(-26, 26);

  /* Range */
  range_init();
  messages::sonar_distance range_msg;
  ros::Publisher range_pub("sonar_distance", &range_msg);
  ros::ServiceServer
    <messages::sonar_toggleRequest, messages::sonar_toggleResponse>
    range_toggle("sonar_toggle", range_toggle_cb);
  ros::ServiceServer
    <messages::sonar_set_scanRequest, messages::sonar_set_scanResponse>
    range_set_scan("sonar_set_scan", range_set_scan_cb);
  nh.advertise(range_pub);
  nh.advertiseService(range_toggle);
  nh.advertiseService(range_set_scan);

  /* BOM */
  bom_init();
  messages::bom bom_msg;
  ros::Publisher bom_pub("bom", &bom_msg);
  nh.advertise(bom_pub);

  /* Headlights (aka Orbs) */
  //orb_init(); TODO debug orbs
  ros::Subscriber<messages::set_headlights> orb_sub("set_headlights",
      orb_callback);
  nh.subscribe(orb_sub);

  /* Cliff sensors */
  cliffSensor_init();
  messages::cliff_status_changed cliff_msg;
  ros::Publisher cliff_pub("cliff", &cliff_msg);
  nh.advertise(cliff_pub);

  id = 0;
  next = 0;
  while (1)
  {
    nh.spinOnce();

    /* Skip loop if the loop period hasn't passed yet */
    /* TODO if we need more exact timing, we can enter a tight loop when now
     * gets close to next, and avoid the uncertainty of nh.spinOnce() */
    now = nh.getHardware()->time();
    if (now < next) {
      continue;
    }
    next = now + MAINLOOP_PERIOD;

    /* Temporary, for BOM testing */
    id++;
    if (id == 0x40) {
      id = 0;
    }
    set_robot_id(id);
    bom_send(0);

    /* BOM */
    for (i = 0; i < 4; i++) {
      int msg = bom_get(i);
      if (msg != BOM_NO_DATA) {
        bom_msg.sender = bom_msg_get_robot_id(msg);
        bom_msg.send_dir = bom_msg_get_dir(msg);
        bom_msg.recv_dir = i;
        bom_pub.publish(&bom_msg);
      }
    }

    /* Stepper / range sensor */
    if (range_enabled) {
      step_sweep();
      range_measure(ranges);
      range_msg.stamp = nh.now();
      range_msg.pos = step_get_pos();
      range_msg.distance0 = ranges[0];
      range_msg.distance1 = ranges[1];
      range_msg.stamp = nh.now();
      range_pub.publish(&range_msg);
    }

    cliff_status = 0;
    if (read_cliffSensor_front())
      cliff_status |= messages::cliff_status_changed::CLIFF_FRONT;
    if (read_cliffSensor_left())
      cliff_status |= messages::cliff_status_changed::CLIFF_LEFT;
    if (read_cliffSensor_right())
      cliff_status |= messages::cliff_status_changed::CLIFF_RIGHT;
    if (cliff_status != cliff_msg.cliff_status) {
      cliff_msg.cliff_status = cliff_status;
      cliff_pub.publish(&cliff_msg);
    }

  }

  return 0;
}
