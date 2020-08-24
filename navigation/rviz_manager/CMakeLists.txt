cmake_minimum_required(VERSION 2.8.3)
project(rviz_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  pcl_ros
  std_msgs
) 

find_package(PCL 1.7 REQUIRED)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


catkin_package(
CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIR} 
)

add_executable(rviz_manager_node src/RvizManager.cc src/libRvizManager.cc)


target_link_libraries(rviz_manager_node 
	${catkin_LIBRARIES}
	${PCL_LIBRARIES} 
)


