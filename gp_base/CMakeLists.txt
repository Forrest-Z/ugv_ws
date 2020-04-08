cmake_minimum_required(VERSION 3.0.2)
project(gp_base)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
) 

find_package(Boost REQUIRED)


Message("Boost Version is " ${Boost_VERSION})
Message("Boost Include is " ${Boost_INCLUDE_DIRS})
Message("Boost Library is " ${Boost_LIBRARIES})

 generate_messages(
   DEPENDENCIES
 )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-narrowing")

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(gp_base_node src/GPBaseNode.cpp src/libGPBaseNode.cpp)

target_link_libraries(gp_base_node ${catkin_LIBRARIES})

