cmake_minimum_required(VERSION 2.8.3)
project(cobot_vision)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
   rospy
   std_msgs
   geometry_msgs
   sensor_msgs
   cv_bridge
)

catkin_package(
   LIBRARIES cobot_vision
   CATKIN_DEPENDS
      rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# target_link_libraries(my_awesome_library ${OpenCV_LIBRARIES})
