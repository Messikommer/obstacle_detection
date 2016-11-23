#include "../include/obstacle_detection.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_detection");
  Obstacle_Detection Obstacle_Detection_object(ros::NodeHandle(),
                                               ros::NodeHandle("~"));

  ros::spin();
  return 0;
}
