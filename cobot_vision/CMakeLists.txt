cmake_minimum_required(VERSION 2.8.3)
project(cobot_vision)

catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
   rospy
   geometry_msgs
   message_generation
   cv_bridge
)

add_service_files(
  FILES
  TagPose.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS
      rospy
      message_runtime
)
