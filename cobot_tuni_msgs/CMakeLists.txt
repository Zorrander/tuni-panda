cmake_minimum_required(VERSION 2.8.3)
project(cobot_tuni_msgs)

find_package(catkin REQUIRED COMPONENTS
   message_generation
)

add_message_files(
  FILES
  Class.msg
  Collection.msg
  Command.msg
  DatatypeProperty.msg
  Individual.msg
  Object.msg
  ObjectProperty.msg
  Test.msg
  Triple.msg
  URI.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)
