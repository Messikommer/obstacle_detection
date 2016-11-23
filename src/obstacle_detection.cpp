#include "../include/obstacle_detection.hpp"

Obstacle_Detection::Obstacle_Detection(const ros::NodeHandle &nh,
                                       const ros::NodeHandle &pnh)
    : nh_(nh),
      pnh_(pnh),
      y_limit_m(1.5),
      tolerance_m(0.2) {
  ROS_INFO("Obstalcle_Detection: initializing");

  obstacle_sub_ = nh_.subscribe("/velodyne_points", 1,
                                &Obstacle_Detection::functionCallback, this);

  obstacle_pub_ = nh_.advertise < sensor_msgs::PointCloud2 > ("/obstacles", 1);

  ROS_INFO("Obstacle_Detection: initialized!");
  subscriber_rate = 0;

  d_histo_ptr[0] = &d_histo_15;
  d_histo_ptr[1] = &d_histo_13;
  d_histo_ptr[2] = &d_histo_11;
  d_histo_ptr[3] = &d_histo_9;
  d_histo_ptr[4] = &d_histo_7;
  d_histo_ptr[5] = &d_histo_5;

}

Obstacle_Detection::~Obstacle_Detection() {
}

void Obstacle_Detection::functionCallback(
    const sensor_msgs::PointCloud2& cloud_message) {
  scan(cloud_message);
}

void Obstacle_Detection::conversion_PC2toPCL(
    const sensor_msgs::PointCloud2& cloud_message,
    pcl::PointCloud<pcl::PointXYZ>& temp_cloud) {

  pcl::PCLPointCloud2 pcl_pc2_1;
  pcl_conversions::toPCL(cloud_message, pcl_pc2_1);
  pcl::fromPCLPointCloud2(pcl_pc2_1, temp_cloud);

}

void Obstacle_Detection::conversion_PCLtoPC2(
    pcl::PointCloud<pcl::PointXYZ>& filtered_cloud,
    sensor_msgs::PointCloud2& converted) {

  pcl::PCLPointCloud2 pcl_pc2_2;
  pcl::toPCLPointCloud2(filtered_cloud, pcl_pc2_2);
  pcl_conversions::fromPCL(pcl_pc2_2, converted);

}

void Obstacle_Detection::histogram_allocation(double d, int j) {
  std::vector < std::vector<double> > &d_histo_temp = *(d_histo_ptr[j]);
//Store
  if (d_histo_temp.size() == 0) {
    std::vector<double> v_dist;
    v_dist.push_back(d);
    v_dist.push_back(1);
    d_histo_temp.push_back(v_dist);
    return;
  } else {
    for (int i = 0; i < d_histo_temp.size(); i++) {
      float a = d_histo_temp[i][0];
      if (a - tolerance_m <= d && d <= a + tolerance_m) {
        //Update the average distance of Intervall and the sum
        d_histo_temp[i][0] = (d_histo_temp[i][1] * d_histo_temp[i][0] + d)
            / (d_histo_temp[i][1] + 1);
        d_histo_temp[i][1] = d_histo_temp[i][1] + 1;
        return;
      }
    }

    //if no interval could be found, creat a new interval
    std::vector<double> v_dist_2;
    v_dist_2.push_back(d);
    v_dist_2.push_back(1);
    d_histo_temp.push_back(v_dist_2);

  }
}

void Obstacle_Detection::Filter(
    pcl::PointCloud<pcl::PointXYZ>& filtered_cloud,
    const pcl::PointCloud<pcl::PointXYZ>& temp_cloud, double* inter_d_ptr) {
  for (int i = 0; i < temp_cloud.size(); i++) {
    double x = temp_cloud.points[i].x;
    double y = temp_cloud.points[i].y;
    double z = temp_cloud.points[i].z;
    //same condition as filter
    if (x > 0 && -y_limit_m < y && y < y_limit_m && z < 0) {

      double check_d = sqrt(x * x + y * y + z * z);
      double alpha_deg = asin(z / check_d) / M_PI * 180;
      bool angle_assigned = false;
      for (int j = 0; (!angle_assigned) && (j < 6); j++) {
        //Check the angle
        if (-15.1 + 2 * j < alpha_deg && alpha_deg < -14.9 + 2 * j) {
          std::vector < std::vector<double> > &d_histo_temp = *(d_histo_ptr[j]);
          if (!(*(inter_d_ptr + j) - tolerance_m < check_d
              && check_d < *(inter_d_ptr + j) + tolerance_m)) {
            filtered_cloud.push_back(temp_cloud.points[i]);
          }
        }
      }  //end of angle check
    }  //end if Range
  }  //end for

}

void Obstacle_Detection::scan(const sensor_msgs::PointCloud2& cloud_message) {
  
  Clock clock;

  pcl::PointCloud < pcl::PointXYZ > filtered_cloud;
  pcl::PointCloud < pcl::PointXYZ > temp_cloud;
  conversion_PC2toPCL(cloud_message, temp_cloud);

  for (int i = 0; i < temp_cloud.size(); i++) {
    bool angle_assigned = false;
    double x = temp_cloud.points[i].x;
    double y = temp_cloud.points[i].y;
    double z = temp_cloud.points[i].z;
    if (x > 0 && -y_limit_m < y && y < y_limit_m && z < 0) {
      double d = sqrt(x * x + y * y + z * z);
      double alpha_deg = asin(z / d) / M_PI * 180;
      for (int j = 0; (!angle_assigned) && (j < 6); j++) {
        if ((-15.1 + 2 * j < alpha_deg) && (alpha_deg < -14.9 + 2 * j)) {
          histogram_allocation(d, j);
          angle_assigned = true;
        }
      }

    }
  }

  int sum[6] = { 0, 0, 0, 0, 0, 0 };  //number of points in the interval
  double inter_d[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };  //distance most points had

  for (int i = 0; i < 6; i++) {
    std::vector < std::vector<double> > &d_histo_temp = *(d_histo_ptr[i]);
    //Search distance intervall, in which most points are found
    for (int j = 0; j < d_histo_temp.size(); j++) {
      if (d_histo_temp[j][1] > sum[i]) {
        sum[i] = d_histo_temp[j][1];
        inter_d[i] = d_histo_temp[j][0];
      }

    }
  }

//std::cout << "sum " << sum[1] << "   size    " << d_histo_15.size()<< "   distance  " << inter_d[1] << std::endl;
  Filter(filtered_cloud, temp_cloud, inter_d);
  sensor_msgs::PointCloud2 converted;
  conversion_PCLtoPC2(filtered_cloud, converted);
  converted.header.stamp = ros::Time::now();
  converted.header.frame_id = cloud_message.header.frame_id;
  obstacle_pub_.publish(converted);

  d_histo_15.clear();
  d_histo_13.clear();
  d_histo_11.clear();
  d_histo_9.clear();
  d_histo_7.clear();
  d_histo_5.clear();
  
  clock.takeTime();
  std::cout<<"Took " << clock.getRealTime() << " ms to filter the input scan."<<std::endl;

}

