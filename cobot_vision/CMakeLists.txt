cmake_minimum_required(VERSION 2.8.3)
project(cobot_vision)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   geometry_msgs
   sensor_msgs
   message_generation
   cv_bridge
)

add_service_files(
  FILES
  TagPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
   LIBRARIES cobot_vision
   CATKIN_DEPENDS
      rospy
      roscpp
      message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# target_link_libraries(my_awesome_library ${OpenCV_LIBRARIES})
