cmake_minimum_required(VERSION 2.8.3)
project(map_manager)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
) 

 generate_messages(
   DEPENDENCIES
 )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(map_manager src/MapManager.cpp src/libMapManager.cpp)

target_link_libraries(map_manager ${catkin_LIBRARIES})
