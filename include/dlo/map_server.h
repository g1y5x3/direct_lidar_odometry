#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Define the point type
using PointType = pcl::PointXYZI;

namespace dlo
{
class MapServer : public rclcpp::Node
{
public:
  MapServer();
  ~MapServer();

  void start();

private:
  void setupMap();
  void publishMapCallback();
  void lidarScanCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  // Global Map
  rclcpp::TimerBase::SharedPtr map_pub_timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  pcl::PointCloud<PointType>::Ptr global_map_;

  // Obstacle Cloud
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
};
} // namespace dlo
