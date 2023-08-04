#include <slip_ira_laser_tools/laser_scan_multi_merger.h>

namespace slip_ira_laser_tools
{

void LaserscanMerger::laserscan_topic_parser()
{
  // LaserScan topics to subscribe
  ros::master::V_TopicInfo topics;

  std::istringstream iss(laserscan_topics);
  std::set<std::string> tokens;
  std::copy(std::istream_iterator<std::string>(iss),
            std::istream_iterator<std::string>(),
            std::inserter<std::set<std::string>>(tokens, tokens.begin()));
  std::vector<std::string> tmp_input_topics;

  while (!tokens.empty())
  {
    ROS_INFO("Waiting for topics ...");
    ros::master::getTopics(topics);
    sleep(1);

    for (int i = 0; i < topics.size(); i++)
    {
      if (topics[i].datatype == "sensor_msgs/LaserScan" && tokens.erase(topics[i].name) > 0)
      {
        tmp_input_topics.push_back(topics[i].name);
      }
    }
  }

  sort(tmp_input_topics.begin(), tmp_input_topics.end());
  std::vector<std::string>::iterator last =
      std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
  tmp_input_topics.erase(last, tmp_input_topics.end());

  // Do not re-subscribe if the topics are the same
  if ((tmp_input_topics.size() != input_topics.size()) ||
      !equal(tmp_input_topics.begin(), tmp_input_topics.end(), input_topics.begin()))
  {

    // Unsubscribe from previous topics
    for (int i = 0; i < scan_subscribers.size(); i++)
      scan_subscribers[i].shutdown();

    input_topics = tmp_input_topics;

    if (input_topics.size() > 0)
    {
      scan_subscribers.resize(input_topics.size());
      clouds_modified.resize(input_topics.size());
      clouds.resize(input_topics.size());
      ROS_INFO("Subscribing to topics\t%ld", scan_subscribers.size());
      for (int i = 0; i < input_topics.size(); ++i)
      {
        scan_subscribers[i] = node_.subscribe<sensor_msgs::LaserScan>(
            input_topics[i].c_str(),
            1,
            boost::bind(&LaserscanMerger::scanCallback, this, _1, input_topics[i]));
        clouds_modified[i] = false;
        std::cout << input_topics[i] << " ";
      }
    }
    else
      ROS_INFO("Not subscribed to any topic.");
  }
}

LaserscanMerger::LaserscanMerger()
{
  ros::NodeHandle nh("~");

  nh.param<std::string>("destination_frame", destination_frame, "cart_frame");
  nh.param<std::string>("cloud_destination_topic", cloud_destination_topic, "/merged_cloud");
  nh.param<std::string>("scan_destination_topic", scan_destination_topic, "/scan_multi");
  nh.param<std::string>("laserscan_topics", laserscan_topics, "");
  nh.param("angle_min", angle_min, -2.36);
  nh.param("angle_max", angle_max, 2.36);
  nh.param("angle_increment", angle_increment, 0.0058);
  nh.param("scan_time", scan_time, 0.0333333);
  nh.param("range_min", range_min, 0.45);
  nh.param("range_max", range_max, 25.0);

  prettyPrintParams();

  this->laserscan_topic_parser();

  point_cloud_publisher_ =
      node_.advertise<sensor_msgs::PointCloud2>(cloud_destination_topic.c_str(), 1, false);
  laser_scan_publisher_ =
      node_.advertise<sensor_msgs::LaserScan>(scan_destination_topic.c_str(), 1, false);
}

void LaserscanMerger::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in,
                                   std::string topic)
{
  sensor_msgs::PointCloud tmpCloud1, tmpCloud2;
  sensor_msgs::PointCloud2 tmpCloud3;

  sensor_msgs::LaserScanPtr scan(new sensor_msgs::LaserScan(*scan_in));

  scan->range_max = very_large_range_ + 1.0;
  for (auto& range : scan->ranges)
  {
    if (std::isinf(range) && range > 0)
    {
      range = very_large_range_;
    }
  }

  // refer to http://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20%28C%2B%2B%29
  try
  {
    // Verify that TF knows how to transform from the received scan to the destination scan frame
    tfListener_.waitForTransform(scan->header.frame_id.c_str(),
                                 destination_frame.c_str(),
                                 scan->header.stamp,
                                 ros::Duration(1));
    projector_.transformLaserScanToPointCloud(scan->header.frame_id,
                                              *scan,
                                              tmpCloud1,
                                              tfListener_,
                                              laser_geometry::channel_option::Distance);
    tfListener_.transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2);
  }
  catch (tf::TransformException ex)
  {
    return;
  }

  for (int i = 0; i < input_topics.size(); i++)
  {
    if (topic.compare(input_topics[i]) == 0)
    {
      sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2, tmpCloud3);
      pcl_conversions::toPCL(tmpCloud3, clouds[i]);
      clouds_modified[i] = true;
    }
  }

  // Count how many scans we have
  int totalClouds = 0;
  for (int i = 0; i < clouds_modified.size(); i++)
    if (clouds_modified[i])
      totalClouds++;

  // Go ahead only if all subscribed scans have arrived
  if (totalClouds == clouds_modified.size())
  {
    pcl::PCLPointCloud2 merged_cloud = clouds[0];
    clouds_modified[0] = false;

    for (int i = 1; i < clouds_modified.size(); i++)
    {
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
      pcl::concatenate(merged_cloud, clouds[i], merged_cloud);
#else
      pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
#endif
      clouds_modified[i] = false;
    }

    point_cloud_publisher_.publish(merged_cloud);

    Eigen::MatrixXf points;
    getPointCloudAsEigen(merged_cloud, points);

    pointcloud_to_laserscan(points, &merged_cloud);
  }
}

void LaserscanMerger::pointcloud_to_laserscan(Eigen::MatrixXf points,
                                              pcl::PCLPointCloud2* merged_cloud)
{
  sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
  output->header = pcl_conversions::fromPCL(merged_cloud->header);
  output->angle_min = this->angle_min;
  output->angle_max = this->angle_max;
  output->angle_increment = this->angle_increment;
  output->time_increment = this->time_increment;
  output->scan_time = this->scan_time;
  output->range_min = this->range_min;
  output->range_max = this->range_max;

  uint32_t ranges_size =
      std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
  output->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());

  float z_sum = 0;
  int z_count = 0;

  for (int i = 0; i < points.cols(); i++)
  {
    const float& x = points(0, i);
    const float& y = points(1, i);
    const float& z = points(2, i);

    if (std::isnan(x) || std::isnan(y) || std::isnan(z))
    {
      ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
      continue;
    }

    double range_sq = pow(y, 2) + pow(x, 2);
    double range_min_sq_ = output->range_min * output->range_min;
    if (range_sq < range_min_sq_)
    {
      ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
                range_sq,
                range_min_sq_,
                x,
                y,
                z);
      continue;
    }

    ROS_INFO_THROTTLE(1.0, "Point: (%f, %f, %f)", x, y, z);

    double angle = atan2(y, x);
    if (angle < output->angle_min || angle > output->angle_max)
    {
      ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n",
                angle,
                output->angle_min,
                output->angle_max);
      continue;
    }
    int index = (angle - output->angle_min) / output->angle_increment;

    if (output->ranges[index] * output->ranges[index] > range_sq or
        std::isnan(output->ranges[index]))
      output->ranges[index] = sqrt(range_sq);
  }

  ROS_WARN("Average Z: %f", z_sum / static_cast<float>(z_count));

  for (auto& range : output->ranges)
  {
    // our manually blown up ranges will violate this
    if (range > this->range_max)
    {
      range = std::numeric_limits<float>::infinity();
    }
  }

  laser_scan_publisher_.publish(output);
}

void LaserscanMerger::prettyPrintParams()
{
  ROS_INFO("LaserScanMerger Parameters:");
  ROS_INFO("  destination_frame: %s", destination_frame.c_str());
  ROS_INFO("  cloud_destination_topic: %s", cloud_destination_topic.c_str());
  ROS_INFO("  scan_destination_topic: %s", scan_destination_topic.c_str());
  ROS_INFO("  laserscan_topics: %s", laserscan_topics.c_str());
  ROS_INFO("  angle_min: %f", angle_min);
  ROS_INFO("  angle_max: %f", angle_max);
  ROS_INFO("  angle_increment: %f", angle_increment);
  ROS_INFO("  scan_time: %f", scan_time);
  ROS_INFO("  range_min: %f", range_min);
  ROS_INFO("  range_max: %f", range_max);
}

} // namespace slip_ira_laser_tools

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_multi_merger");

  slip_ira_laser_tools::LaserscanMerger _laser_merger;
  ros::spin();

  return 0;
}
