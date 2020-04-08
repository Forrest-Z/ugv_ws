cmake_minimum_required(VERSION 2.8.3)
project(mission_control)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    pcl_ros
) 
find_package(PCL 1.7 REQUIRED)



catkin_package(
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


include_directories(
	include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIR} 
)


add_library(${PROJECT_NAME} 
 	include/Planning.cc
 	include/Controlling.cc
	include/Routing.cc
	include/Tools.cc
	src/libMissionControl.cc
)
target_link_libraries(${PROJECT_NAME}   
${catkin_LIBRARIES} 
${PCL_LIBRARIES}  
)


add_executable(mission_control_node src/MissionControl.cc)
target_link_libraries(mission_control_node ${PROJECT_NAME} )

