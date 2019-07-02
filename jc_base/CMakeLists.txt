cmake_minimum_required(VERSION 3.0.2)
project(jc_base)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
) 

find_package(Boost 1.70 REQUIRED PATHS "/usr/local/boost-1.70/lib/cmake/Boost-1.70.0")

# find_package(Boost PATHS "/usr/local/boost-1.70/lib/cmake/Boost-1.70.0")

set(Boost_INCLUDE_DIRS /usr/local/boost-1.70/include)
set(Boost_LIBRARIES /usr/local/boost-1.70/lib)



Message("Boost Version is " ${Boost_VERSION})
Message("Boost Include is " ${Boost_INCLUDE_DIRS})
Message("Boost Library is " ${Boost_LIBRARIES})

 generate_messages(
   DEPENDENCIES
 )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(jc_base_node src/JCBaseNode.cpp src/libJCBaseNode.cpp)

target_link_libraries(jc_base_node ${catkin_LIBRARIES})

