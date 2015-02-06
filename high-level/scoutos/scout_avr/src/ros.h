#ifndef _ROS_H_
#define _ROS_H_

#include "Atmega128rfa1.h"

namespace ros
{
  typedef ros::NodeHandle_<Atmega128rfa1, MAX_SUBSCRIBERS, MAX_PUBLISHERS,
          INPUT_SIZE, OUTPUT_SIZE> NodeHandle;
}

#endif
