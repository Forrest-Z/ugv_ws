cmake_minimum_required(VERSION 2.8.3)
project(log_manager)

find_package(catkin REQUIRED COMPONENTS
    roscpp
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
)

add_executable(log_manager_node src/LogManager.cc src/libLogManager.cc)

target_link_libraries(log_manager_node ${catkin_LIBRARIES})
