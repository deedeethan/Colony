cmake_minimum_required(VERSION 2.4.6)
project(messages)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

add_service_files(
    FILES
    query_cliff.srv
    query_encoders.srv
    query_linesensor.srv
    query_metal_detector.srv
    query_motors.srv
    query_paintboard.srv
    query_power.srv
    reset_encoders.srv
    sonar_set_scan.srv
    sonar_toggle.srv
)

add_message_files(
   FILES
   ScoutPosition.msg
   WirelessPacket.msg
   bom.msg
   button_event.msg
   cliff_status_changed.msg
   power_state_changed.msg
   set_headlights.msg
   set_motors.msg
   set_paintboard.msg
   sonar_distance.msg
 )
generate_messages(
   DEPENDENCIES
   std_msgs
)
catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)
#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#catkin_package()
