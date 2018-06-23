cmake_minimum_required(VERSION 2.8.3)
project(ml_msgs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

## Declare ROS messages and services
add_message_files(
  DIRECTORY msg
  FILES
  Marker.msg
  MarkerDetection.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
#  DEPENDS
#  ml_msgs_generate_messages_cpp
)
