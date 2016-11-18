
/*Detects obstacles, which are on the road.  Evaluates the Distance of the Points published by the Laser and publishes points with a certain distance difference as obstacles. */

#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>


#include <iostream>
#include <vector>

#define PI 3.14159265

ros::Publisher obstacle_pub;


void scan(const sensor_msgs::PointCloud2& cloud_message) {
  
  
  //converts sensor_msgs::PointCloud2 to pcl::PCLPointCLoud2 and then to 
  pcl::PCLPointCloud2 pcl_pc2_1;
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  pcl_conversions::toPCL(cloud_message, pcl_pc2_1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2_1, *temp_cloud); 

//store numbers of points and the distance interval, in which the points can be found
  std::vector< std::vector<float> > d_histo;
  
  
  //Filter
  
 for(int i_1=0;i_1 <= temp_cloud->size(); i_1++) {

   float x=temp_cloud->points[i_1].x;
   float y=temp_cloud->points[i_1].y;
   float z=temp_cloud->points[i_1].z;
   
   //check if in Range
    if (x>0 && -1.5<y && y<1.5 && z<0) {
      
      float d=sqrt(x*x+y*y+z*z);
      float alpha=asin(z/d)/PI*180;

      
      //Check the angle
      if(-13.1<alpha && alpha<-12.9) {
        
        
        //Store
        if(d_histo.size()==0) {
          std::vector<float> v_dist;
          v_dist.push_back(d);
          v_dist.push_back(1);
          d_histo.push_back(v_dist);
          
        }
        
        else{
          bool included=false;
           for(int i_2=0; i_2<d_histo.size(); i_2++ ) {
             float a=d_histo[i_2][0];
             
             
             if(a-0.06<= d && d<=a+0.06) {    
               //Update the average distance of Intervall and the sum
               d_histo[i_2][0]=(d_histo[i_2][1]*d_histo[i_2][0]+d)/(d_histo[i_2][1]+1);
               d_histo[i_2][1]=d_histo[i_2][1]+1;
               included=true;
 
               
             }
         
           }
           
           //if no interval could be found, creat a new interval
           if(!included) {
               std::vector<float> v_dist_2;
               v_dist_2.push_back(d);
               v_dist_2.push_back(1);
               d_histo.push_back(v_dist_2);
           }
           
        }
        

        
        }  //End of angle check

      } //End of range check 

    } //End of Filter  
 

 
 //Search distance intervall, in which most points are found
int sum=0;  //number of points in the interval
float inter_d=0.0;  //distance most points had
 
 for(int i_3=0; i_3<d_histo.size(); i_3++) {
   if(d_histo[i_3][1]>sum){
     sum=d_histo[i_3][1];
     inter_d=d_histo[i_3][0];
   }
   
   
 }
 
 std::cout<<inter_d<<"   sum:    "<<sum<<"   size:   "<<d_histo.size()<<"    "<<std::endl;
  
 
 
 
 
 
 
 //Second Iteratation to find the points with the distance, which the most points had.
 
 for(int i_4=0;i_4 <= temp_cloud->size(); i_4++) {
   
   float x=temp_cloud->points[i_4].x;
   float y=temp_cloud->points[i_4].y;
   float z=temp_cloud->points[i_4].z; 
   
//same condition as filter
  if (x>0 && -1.5<y && y<1.5 && z<0) {
 
    float check_d=sqrt(x*x+y*y+z*z);
    float alpha=asin(z/check_d)/PI*180;
    
    //Check the angle
    if(-13.1<alpha && alpha<-12.9) {


     if (!(inter_d-0.06<check_d && check_d<inter_d+0.06)) {
       filtered_cloud.push_back(temp_cloud->points[i_4]); 



       
     }
   }
  }
   
 } //End second Iteration
 
 
 //convertes bach to sensor_msgs::PointCloud2. Just for Visualization.
  sensor_msgs::PointCloud2 converted;
  sensor_msgs::PointCloud2* converted_ptr=&converted;
  pcl::PCLPointCloud2 pcl_pc2_2;
  pcl::toPCLPointCloud2(filtered_cloud, pcl_pc2_2);
  
  pcl_conversions::fromPCL(pcl_pc2_2, *converted_ptr);
  
  converted.header.stamp = ros::Time::now();
  converted.header.frame_id = cloud_message.header.frame_id;
  
  
  obstacle_pub.publish(converted);
  
 
  
    
  
}  //End of Scan







int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_detection");
  ros::NodeHandle n;


  
  
  
  obstacle_pub = n.advertise<sensor_msgs::PointCloud2>("/obstacles", 1000);
  ros::Subscriber sub = n.subscribe("/velodyne_points", 1000, scan);

  
  
  
  
  ros::spin();
  return 0;
}




