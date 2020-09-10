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
    rospy
    std_msgs
    geometry_msgs
    tf
    tf_conversions
    eigen_conversions
    message_generation
    moveit_core
    moveit_ros_planning_interface
)

catkin_python_setup()

catkin_package(
   LIBRARIES cobot_controllers
   CATKIN_DEPENDS
      controller_interface
      hardware_interface
      pluginlib
      franka_hw
      roscpp
      message_runtime
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

add_executable(robot_model_and_robot_state_tutorial
    src/robot_model_and_robot_state_tutorial.cpp
)

add_library(cobot_controllers
    src/cartesian_pose_controller.cpp
    src/cartesian_velocity_controller.cpp
    src/joint_position_controller.cpp

)

target_link_libraries(cobot_controllers ${catkin_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(robot_model_and_robot_state_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
