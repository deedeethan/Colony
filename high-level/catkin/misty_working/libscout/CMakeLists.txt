cmake_minimum_required(VERSION 2.4.6)
project(libscout)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

catkin_package(
    DEPENDS messages
)
# Generate BehaviorList files and scoutsim files.
message(${PROJECT_SOURCE_DIR})
#add_custom_command(OUTPUT src/BehaviorList.cpp COMMAND ${PROJECT_SOURCE_DIR}/generate_behavior_lists.py)
set(MAIN_FILES src/Sensors.cpp src/Behavior.cpp src/BehaviorList.cpp src/BehaviorProcess.cpp)
FILE(GLOB BEHAVIOR_FILES "${PROJECT_SOURCE_DIR}/src/behaviors/*.cpp")
FILE(GLOB TEST_BEHAVIOR_FILES "${PROJECT_SOURCE_DIR}/src/test_behaviors/*.cpp")
FILE(GLOB HELPER_FILES "${PROJECT_SOURCE_DIR}/src/helper_classes/*.cpp")
FILE(GLOB CONTROL_CLASSES "${PROJECT_SOURCE_DIR}/src/*Control.cpp")
set(WIRELESS_CONTROL_CLASSES src/WirelessSender.cpp src/WirelessReceiver.cpp)

include_directories(messages)

## specify libraries to link a library or executable target against
add_executable(libscout ${MAIN_FILES} ${CONTROL_CLASSES} ${WIRELESS_CONTROL_CLASSES} ${HELPER_FILES} ${BEHAVIOR_FILES} ${TEST_BEHAVIOR_FILES})
set_target_properties(libscout PROPERTIES COMPILE_FLAGS "-std=c++0x")
target_link_libraries(libscout
   ${catkin_LIBRARIES})
