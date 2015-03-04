cmake_minimum_required(VERSION 2.4.6)
project(scoutsim)
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

generate_messages(
    DEPENDENCIES
    std_msgs
)

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )

find_package(Boost REQUIRED COMPONENTS components)
add_executable(scoutsim_node src/scoutsim.cpp src/scout.cpp src/ghost_scout.cpp src/sim_frame.cpp)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(scoutsim_node ${wxWidgets_LIBRARIES})
