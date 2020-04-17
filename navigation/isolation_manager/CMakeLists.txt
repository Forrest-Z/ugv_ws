cmake_minimum_required(VERSION 2.8.3)
project(isolation_manager)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    pcl_ros
) 

find_package(PCL 1.7 REQUIRED)

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

add_executable(isolation_manager_node src/IsolationManager.cc src/libIsolationManager.cc)

target_link_libraries(isolation_manager_node ${catkin_LIBRARIES})
