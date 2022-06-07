cmake_minimum_required(VERSION 3.0.2)
project(enamour_core)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2_ros
)



catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)