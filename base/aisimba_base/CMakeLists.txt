cmake_minimum_required(VERSION 2.8.3)
project(aisimba_base)

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

add_executable(base_node src/AisimbaBase.cpp src/libAisimbaBase.cpp)

target_link_libraries(base_node ${catkin_LIBRARIES})

