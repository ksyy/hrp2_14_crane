cmake_minimum_required(VERSION 2.8.3)

project(hrp2_14_crane)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    sensor_msgs
)

find_package(PkgConfig REQUIRED)
pkg_search_module(ftdi REQUIRED libftdi)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${ftdi_INCLUDE_DIRS}
)

add_executable(crane_listener src/crane_listener.cpp 
  src/crane_usb.cpp
  src/crane_message_handler.cpp)
target_link_libraries(crane_listener ${ftdi_LIBRARIES} ${catkin_LIBRARIES})
