cmake_minimum_required(VERSION 2.8.3)
project(speed_test)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES speed_test
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
)




add_executable(speed_control src/speed_control.cpp)

target_link_libraries(speed_control ${catkin_LIBRARIES})
