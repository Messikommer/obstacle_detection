cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detection)


find_package(catkin REQUIRED COMPONENTS
	sensor_msgs
	roscpp
 	pcl_conversions
  	pcl_ros
)



include_directories(  
	${catkin_INCLUDE_DIRS}  
	${PCL_INCLUDE_DIRS}
	)
	
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(obstacle_detection ${PROJECT_SOURCE_DIR}/src/obstacle_detection.cpp ${PROJECT_SOURCE_DIR}/src/obstacle_detection_node.cpp)

target_link_libraries(obstacle_detection 
  ${catkin_LIBRARIES}
  ${roslib_LIBRARIES}
  ${PCL_LIBRARIES}
)

