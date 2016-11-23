#include <math.h>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

class Obstacle_Detection {
 public:
  Obstacle_Detection(const ros::NodeHandle & nh, const ros::NodeHandle & pnh);
  ~Obstacle_Detection();
  void conversion_PC2toPCL(const sensor_msgs::PointCloud2& cloud_message,
                           pcl::PointCloud<pcl::PointXYZ>& temp_cloud);
  void conversion_PCLtoPC2(pcl::PointCloud<pcl::PointXYZ>& filtered_cloud,
                           sensor_msgs::PointCloud2& converted);

  void functionCallback(const sensor_msgs::PointCloud2& cloud_message);

  void scan(const sensor_msgs::PointCloud2& cloud_message);

  void histogram_allocation(double d, int j);
  void Filter(pcl::PointCloud<pcl::PointXYZ>& filtered_cloud,
              const pcl::PointCloud<pcl::PointXYZ>& temp_cloud,
              double* inter_d_ptr);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher obstacle_pub_;
  ros::Subscriber obstacle_sub_;
  std::vector<std::vector<double> > d_histo_15;
  std::vector<std::vector<double> > d_histo_13;
  std::vector<std::vector<double> > d_histo_11;
  std::vector<std::vector<double> > d_histo_9;
  std::vector<std::vector<double> > d_histo_7;
  std::vector<std::vector<double> > d_histo_5;
  std::vector<std::vector<double> >* d_histo_ptr[6];
  double y_limit_m;
  double tolerance_m;
  int subscriber_rate;

};

class Clock {

 public:
  Clock() { start(); }

  /// \brief Start clock timer.
  void start(){
    gettimeofday(&real_time_start_, NULL);
    cpu_start_ = clock();
  }

  /// \brief Sample clock timer.
  void takeTime(){
    struct timeval end;
    gettimeofday(&end, NULL);
    cpu_time_ms_ = double(clock() - cpu_start_) / CLOCKS_PER_SEC * kSecondsToMiliseconds;

    long seconds, useconds;

    seconds  = end.tv_sec  - real_time_start_.tv_sec;
    useconds = end.tv_usec - real_time_start_.tv_usec;
    real_time_ms_ = (seconds * kSecondsToMiliseconds +
        useconds * kMicrosecondsToMiliseconds) + 0.5;
  }

  /// \brief Return elapsed physical time.
  double getRealTime() { return real_time_ms_; }

  /// \brief Return elapsed CPU time.
  double getCPUTime() { return cpu_time_ms_; }

  double takeRealTime() { takeTime(); return getRealTime(); }

 private:
  struct timeval real_time_start_;
  double real_time_ms_, cpu_time_ms_;
  clock_t cpu_start_;

  static const double kSecondsToMiliseconds = 1000.0;
  static const double kMicrosecondsToMiliseconds = 0.001;
};