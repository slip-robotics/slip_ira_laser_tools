#ifndef LASERSCAN_MERGER_H
#define LASERSCAN_MERGER_H

#include "pcl_ros/point_cloud.h"
#include "sensor_msgs/LaserScan.h"
#include <Eigen/Dense>
#include <laser_geometry/laser_geometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <string.h>
#include <tf/transform_listener.h>

namespace slip_ira_laser_tools
{

class LaserscanMerger
{
public:
  LaserscanMerger();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic);
  void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2* merged_cloud);

private:
  ros::NodeHandle node_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener tfListener_;

  ros::Publisher point_cloud_publisher_;
  ros::Publisher laser_scan_publisher_;
  std::vector<ros::Subscriber> scan_subscribers;
  std::vector<bool> clouds_modified;

  std::vector<pcl::PCLPointCloud2> clouds;
  std::vector<std::string> input_topics;

  void laserscan_topic_parser();

  double angle_min;
  double angle_max;
  double angle_increment;
  double time_increment;
  double scan_time;
  double range_min;
  double range_max;

  std::string destination_frame;
  std::string cloud_destination_topic;
  std::string scan_destination_topic;
  std::string laserscan_topics;

  float very_large_range_{1e6f};
};

} // namespace slip_ira_laser_tools

#endif // LASERSCAN_MERGER_H
