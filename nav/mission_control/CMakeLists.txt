cmake_minimum_required(VERSION 2.8.3)
project(mission_control)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
		PCL
		pcl_ros
) 

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)

 generate_messages(
   DEPENDENCIES
 )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIR} 
)

add_executable(speed_manager src/speed_manager/SpeedManager.cpp src/speed_manager/libSpeedManager.cpp)
add_executable(navi_manager src/navi_manager/NaviManager.cpp src/navi_manager/libNaviManager.cpp)


target_link_libraries(speed_manager ${OpenCV_LIBS}  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(navi_manager ${OpenCV_LIBS}  ${catkin_LIBRARIES})

