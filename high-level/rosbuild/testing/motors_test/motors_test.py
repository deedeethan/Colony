#!/usr/bin/python

import roslib; roslib.load_manifest('motors_test')
import rospy
from motors.msg import set_motors

if __name__ == '__main__':
  rospy.init_node('motors_test')

  pub = rospy.Publisher('/set_motors', set_motors)
  msg = set_motors(fl_set=True, fl_speed=0)
  fl_ds = 5

  while not rospy.is_shutdown():

    msg.fl_speed += fl_ds

    if msg.fl_speed >= 100:
      fl_ds = -5
      msg.fl_speed = 100
    elif msg.fl_speed <= -100:
      fl_ds = 5
      msg.fl_speed = -100

    pub.publish(msg)

    rospy.sleep(0.05)
