cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detection)


find_package(catkin REQUIRED
sensor_msgs
)



add_executable(obstacle_detection obstacle_detection.cpp)
target_link_libraries(obstacle_detection ${catkin_LIBRARIES})

