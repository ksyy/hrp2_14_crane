cmake_minimum_required(VERSION 2.4.6)
SET(CMAKE_VERBOSE_MAKEFILE ON)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#include(/opt/ros/electric/ros/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

# Crane listener
rosbuild_add_executable(crane_listener src/crane_listener.cpp 
  src/crane_usb.cpp
  src/crane_message_handler.cpp)
target_link_libraries(crane_listener ftdi)
#target_link_libraries(crane_listener efence)
