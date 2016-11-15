#include <ros/ros.h>
#include <math.h>
#include "sensor_msgs/PointField.h"
#include <sensor_msgs/PointCloud2.h>


void scan(const sensor_msgs::PointCloud2ConstPtr &PCL2) {
  float x=PCL2->data[1];
  float y=PCL2->data[2];
  float z=PCL2->data[3];
  float d=sqrt(y*y+x*x);
  
  
  float alpha=atan(z/d);
  float beta=atan(y/d);
  
  if(-16<alpha<16 && -5<beta<5 ) {
    
    
  }
  
  else if(-5<beta<5){
    
    
  }
  

  
  
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_detection");
  ros::NodeHandle nh;
  
  ross::
  
  
  
  
  
  ros::spin();

  return 0;
}




