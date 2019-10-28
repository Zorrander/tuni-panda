cmake_minimum_required(VERSION 2.8.3)
project(cobot_controllers)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    pluginlib
    franka_hw
    roscpp
)

catkin_package(
   LIBRARIES cobot_controllers
   CATKIN_DEPENDS
      controller_interface
      hardware_interface
      pluginlib
      franka_hw
      roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(cobot_controllers
    src/cartesian_pose_controller.cpp
)
