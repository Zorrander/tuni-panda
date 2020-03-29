cmake_minimum_required(VERSION 2.8.3)
project(sem_server_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  cobot_tuni_msgs
)

catkin_python_setup()


add_service_files(
   FILES
   Ask.srv
   Describe.srv
   Select.srv
   Update.srv
   GenerateInstanceName.srv
   CreateInstance.srv
)

generate_messages(DEPENDENCIES
                  cobot_tuni_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sem_server_ros
   CATKIN_DEPENDS rospy
                  message_runtime
#  DEPENDS jena_sempy
)

catkin_add_nosetests(test)
