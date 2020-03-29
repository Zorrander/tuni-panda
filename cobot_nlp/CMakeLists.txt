cmake_minimum_required(VERSION 2.8.3)
project(cobot_nlp)

catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
   rospy
)

catkin_package(
   CATKIN_DEPENDS
      rospy
)
