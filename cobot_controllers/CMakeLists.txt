cmake_minimum_required(VERSION 2.8.3)
project(cobot_controllers)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    franka_example_controllers
    pluginlib
    franka_hw
    roscpp
    rospy
    genmsg
    tf
    tf_conversions
    eigen_conversions
    moveit_core
    moveit_ros_planning_interface
)

catkin_python_setup()


catkin_package(
   LIBRARIES cobot_controllers
   CATKIN_DEPENDS
      controller_interface
      hardware_interface
      franka_example_controllers
      pluginlib
      franka_hw
      roscpp
      tf
      tf_conversions
      eigen_conversions
      moveit_core
      moveit_ros_planning_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(cobot_controllers
    src/joint_velocity_controller.cpp
    src/cartesian_pose_controller.cpp
    src/joint_position_controller.cpp
    # src/cartesian_impedance_controller.cpp
    # src/joint_impedance_controller.cpp
)


target_link_libraries(cobot_controllers ${catkin_LIBRARIES} ${Boost_LIBRARIES})

