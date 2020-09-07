cmake_minimum_required(VERSION 2.8.3)
project(cobot_msgs)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    message_generation
)


add_service_files(
  FILES
  ReachJointPose.srv
  ReachCartesianPose.srv
  NamedTarget.srv
  RobotSpeed.srv
  MoveGripper.srv
  ListTargets.srv
  Grasp.srv
  TagPose.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
   CATKIN_DEPENDS
     std_msgs
     geometry_msgs
     message_runtime
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
