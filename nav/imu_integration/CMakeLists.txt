cmake_minimum_required(VERSION 2.8.3)
project(imu_integration)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${EIGEN3_INCLUDE_DIR} 
	${PCL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS 
)


add_executable(imu_integ src/imu_integ.cpp src/utility/utility.cpp )
target_link_libraries(imu_integ ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(rspoints_test src/rspoints_test.cpp)
target_link_libraries(rspoints_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(pointclou_to_laserscan src/pointclou_to_laserscan.cpp)
target_link_libraries(pointclou_to_laserscan ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(tf_pub src/tf_pub.cpp)
target_link_libraries(tf_pub ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})




