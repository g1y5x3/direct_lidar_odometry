#include "dlo/dlo.h"
#include "dlo/utils.h"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

// ROS Messages and TF2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
// #include <tf2_sensors_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.hpp>

// Nano GCIP
#include <nano_gicp/nano_gicp.hpp>

typedef pcl::PointXYZI PointType;

class dlo::LocalizationNode : public rclcpp::Node {

public:
  LocalizationNode();
  ~LocalizationNode();

  void start();

private:
  void getinitParams();
  void loadGlobalMap();
  void setupGICP();
  void publishTransform(const rclcpp::Time& stamp);
  void debug();

  // ROS Callback Functions
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg);

  // ROS Members
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  // GICP and PCL Members
  nano_gicp::NanoGICP<PointType, PointType> gicp_;
  pcl::PointCloud<PointType>::Ptr current_scan_;
  pcl::PointCloud<PointType>::Ptr global_map_;

  // State and Threading Members
  std::atomic<bool> is_initialized_;
  std::mutex icp_mutex_;
  Eigen::Matrix4f T_odom_baselink_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f T_map_odom_ = Eigen::Matrix4f::Identity();

  // Parameters
  bool initial_pose_use_;
  Eigen::Vector3f initial_position_;
  Eigen::Quaternionf initial_orientation_;
};
