cmake_minimum_required(VERSION 2.8.3)
project(sem_server_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
   FILES
   DatatypeProperty.msg
   Individual.msg
   ObjectProperty.msg
   Triple.msg
   URI.msg
)

add_service_files(
   FILES
   CreateSem.srv
   ReadSem.srv
   UpdateSem.srv
   DeleteSem.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sem_server_ros
   CATKIN_DEPENDS rospy std_msgs message_runtime
#  DEPENDS system_lib
)
