cmake_minimum_required(VERSION 2.8.3)
project(can_node)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        rover_msgs
        )

catkin_package(
	CATKIN_DEPENDS
	roscpp
	std_msgs
	rover_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(include)

add_executable(can_node src/main.cpp src/cansend.cpp src/lib.cpp src/candump.cpp)


add_dependencies(can_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(can_node ${catkin_LIBRARIES})
