#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>


ros::Publisher obstacle_pub;




void scan(const sensor_msgs::PointCloud2ConstPtr &PCL2) {
  //float x=PCL2->data[100];
  //float y=PCL2->data[2];
  //float z=PCL2->data[3];
  std::cout<<PCL2->row_step<<std::endl;
  
  
  

  
  /*
  
  
  
  float d1=sqrt(y*y+x*x+z*z);
  float d2=sqrt(y*y+x*x);
  


 
  
  float alpha=atan(z/d2);
  float beta=atan(y/d2);
  
  if(-16<alpha<-14 && 0<beta<10 ) {
    obstacle_pub.publish(PCL2);  }
  
  else if(-10<beta<10) {}
  
  */
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_detection");
  ros::NodeHandle n;

  
  obstacle_pub = n.advertise<sensor_msgs::PointCloud2>("/obstacles", 1000);
  ros::Subscriber sub = n.subscribe("/velodyne_points", 1000, scan);

  
  
  
  
  ros::spin();
  return 0;
}

//sensor_msgs::PointCloud2 p;
//p.data[1]=PCL2->data[1];




