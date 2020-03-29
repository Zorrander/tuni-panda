cmake_minimum_required(VERSION 2.8.3)
project(cobot_control)

find_package(catkin REQUIRED COMPONENTS
    rospy
    message_generation
    actionlib_msgs
)

catkin_python_setup()

add_service_files(
  FILES
  ReachTag.srv
)

generate_messages()

catkin_package(
   CATKIN_DEPENDS
   rospy
   message_runtime
)
