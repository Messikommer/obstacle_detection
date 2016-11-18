
/*Detects obstacles, which are on the road.  Evaluates the Distance of the Points published by the Laser and publishes points with a certain distance difference as obstacles. */

#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

ros::Publisher obstacle_pub;



void scan(const sensor_msgs::PointCloud2& PCL2) {
  
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
  pcl_conversions::toPCL(PCL2, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  

  for(int i=0;i <= temp_cloud->size(); i++) {
    if (temp_cloud->points[i].x>0 && i<temp_cloud->size()) {
      filtered_cloud->points[i]=temp_cloud->points[i];
    }  
  }
  
  sensor_msgs::PointCloud2* converted;
  
  //pcl::toROSMsg( &filtered_cloud, &converted )   
  
    
  
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




