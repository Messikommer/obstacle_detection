cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)



add_executable(obstacle_detection obstacle_detection.cpp)
target_link_libraries(obstacle_detection ${catkin_LIBRARIES})

