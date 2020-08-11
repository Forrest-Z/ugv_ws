cmake_minimum_required(VERSION 3.0.2)
project(mqtt_ros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
) 

find_package(Boost REQUIRED)

set(MOSQUITTOPP_LIBRARIES
     /usr/local/lib/mqtt/libmosquitto.so
     /usr/local/lib/mqtt/libmosquitto.so.1
     /usr/local/lib/mqtt/libmosquittopp.so
     /usr/local/lib/mqtt/libmosquittopp.so.1
)
set(MOSQUITTOPP_INCLUDE_DIR
     /usr/local/include/mqtt/
)

 generate_messages(
   DEPENDENCIES
 )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-narrowing -Wno-deprecated-declarations")

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${MOSQUITTOPP_INCLUDE_DIR}
)

add_executable(mqtt_ros_node src/MqttNode.cpp src/libMqttNode.cpp)

target_link_libraries(mqtt_ros_node ${catkin_LIBRARIES} ${MOSQUITTOPP_LIBRARIES})

