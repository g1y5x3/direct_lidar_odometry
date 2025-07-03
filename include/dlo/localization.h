#include "dlo/dlo.h"
#include "dlo/utils.h"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS Messages and TF2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

// Nano GCIP
#include <nano_gicp/nano_gicp.hpp>

// Standard Libraries
#include <optional>

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
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // ROS Members
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // GICP and PCL Members
  nano_gicp::NanoGICP<PointType, PointType> gicp_;
  pcl::PointCloud<PointType>::Ptr current_scan_;
  pcl::PointCloud<PointType>::Ptr global_map_;

  // State and Threading Members
  Eigen::Matrix4f T_map_odom_, T_prev_map_odom, T_odom_base_;
  std::optional<geometry_msgs::msg::Pose> latest_odom_pose_;
  std::atomic<bool> is_initialized_;
  std::mutex odom_mutex_;

  // Parameters
  bool initial_pose_use_;
  Eigen::Vector3f initial_position_;
  Eigen::Quaternionf initial_orientation_;
};