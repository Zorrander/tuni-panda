cmake_minimum_required(VERSION 2.8.3)
project(cobot_control)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
    actionlib_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  Test.msg
)

add_service_files(
  FILES
  ReachTag.srv
)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(
   LIBRARIES cobot_controllers
   CATKIN_DEPENDS
   message_runtime
   actionlib_msgs
)
