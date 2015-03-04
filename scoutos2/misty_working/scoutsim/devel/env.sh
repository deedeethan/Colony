#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/yuyang/catkin_ws/src/scoutsim/devel', type 'exit' to leave"
  . "/home/yuyang/catkin_ws/src/scoutsim/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/yuyang/catkin_ws/src/scoutsim/devel'"
else
  . "/home/yuyang/catkin_ws/src/scoutsim/devel/setup.sh"
  exec "$@"
fi
