cmake_minimum_required(VERSION 2.4.6)
project(buttons)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS messages
)
include_directories(messages)
add_executable(buttons_node src/buttons.cpp)
target_link_libraries(encoders_node
   ${catkin_libraries}
)
