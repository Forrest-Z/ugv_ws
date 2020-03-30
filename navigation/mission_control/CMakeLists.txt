cmake_minimum_required(VERSION 2.8.3)
project(mission_control)

ADD_DEFINITIONS(-DLINUX)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
		pcl_ros
) 

find_package(OpenCV 2 REQUIRED PATHS "/usr/local/opencv-2.4/share/OpenCV")
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(realsense2 REQUIRED)

catkin_package(
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


include_directories(
	include
	exc
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIR} 
	${realsense2_INCLUDE_DIR}
	mtcnn_model
	feature_model
)
LINK_DIRECTORIES(./lib)

add_library(${PROJECT_NAME} 
 	include/Planning.cc
 	include/Controlling.cc
	include/Routing.cc
	include/Tools.cc
	src/libMissionControl.cc
)
target_link_libraries(${PROJECT_NAME}  
${OpenCV_LIBS}  
${catkin_LIBRARIES} 
${PCL_LIBRARIES}  
${realsense2_LIBRARY}
mxnet

)
AUX_SOURCE_DIRECTORY(exc DIR_SRCS)


add_executable(mission_control_node src/MissionControl.cc ${DIR_SRCS})
target_link_libraries(mission_control_node ${PROJECT_NAME} )

