#include "dlo/localization.h"
#include "dlo/utils.h" // Assuming a utils file for conversions like poseMsgToEigen

namespace dlo {

LocalizationNode::LocalizationNode() : Node("dlo_localization_node"), is_initialized_(false) {
  RCLCPP_INFO(this->get_logger(), "Initializing DLO Localization Node...");

  this->getParams();

  // Load the global map
  this->global_map_ = std::make_shared<pcl::PointCloud<PointType>>();
  if (pcl::io::loadPCDFile<PointType>(this->map_path_, *this->global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load map file from %s", this->map_path_.c_str());
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Map loaded successfully with %ld points.", this->global_map_->size());

  // Setup GICP
  this->setupGicp();

  // Initialize ROS publishers and subscribers
  this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&LocalizationNode::odomCallback, this, std::placeholders::_1));

  this->pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", rclcpp::QoS(1), std::bind(&LocalizationNode::pointcloudCallback, this, std::placeholders::_1));

  this->initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10, std::bind(&LocalizationNode::initialPoseCallback, this, std::placeholders::_1));
      
  RCLCPP_INFO(this->get_logger(), "DLO Localization Node Initialized. Waiting for initial pose...");
}

LocalizationNode::~LocalizationNode() {}

void LocalizationNode::getParams() {
  this->declare_parameter<std::string>("map_path", "map.pcd");
  this->map_path_ = this->get_parameter("map_path").as_string();
  
  this->declare_parameter<bool>("voxel_filter.use", true);
  this->vf_scan_use_ = this->get_parameter("voxel_filter.use").as_bool();

  this->declare_parameter<double>("voxel_filter.res", 0.1);
  this->vf_scan_res_ = this->get_parameter("voxel_filter.res").as_double();
  
  // You can add GICP parameters here as well if you want them to be configurable
}

void LocalizationNode::setupGicp() {
  this->vf_scan_.setLeafSize(this->vf_scan_res_, this->vf_scan_res_, this->vf_scan_res_);
  
  // Set GICP parameters (copy from your odom.cc for consistency)
  this->gicp_.setCorrespondenceRandomness(20);
  this->gicp_.setMaxCorrespondenceDistance(1.0);
  this->gicp_.setMaximumIterations(64);
  this->gicp_.setTransformationEpsilon(1e-3);

  // Set the map as the permanent target for GICP
  this->gicp_.setInputTarget(this->global_map_);

  // Pre-compute covariances for the entire map (critical performance step)
  RCLCPP_INFO(this->get_logger(), "Pre-calculating GICP target covariances...");
  this->gicp_.calculateTargetCovariances();
  RCLCPP_INFO(this->get_logger(), "GICP setup complete.");
}

void LocalizationNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    const auto& p = msg->pose.pose.position;
    const auto& q = msg->pose.pose.orientation;
    Eigen::Translation3f translation(p.x, p.y, p.z);
    Eigen::Quaternionf rotation(q.w, q.x, q.y, q.z);
    
    std::lock_guard<std::mutex> lock(this->odom_mutex_);
    this->T_map_odom_ = (translation * rotation).matrix();
    this->is_initialized_ = true;

    RCLCPP_INFO(this->get_logger(), "Localization initialized via RViz at [x: %.2f, y: %.2f]", p.x, p.y);
}

void LocalizationNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(this->odom_mutex_);
  this->latest_odom_pose_ = msg->pose.pose;
}

void LocalizationNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg) {
  if (!this->is_initialized_) {
    return;
  }

  std::unique_lock<std::mutex> lock(this->odom_mutex_);
  if (!this->latest_odom_pose_) {
    RCLCPP_WARN(this->get_logger(), "No odometry received yet. Skipping localization.");
    return;
  }
  geometry_msgs::msg::Pose odom_pose = *this->latest_odom_pose_;
  Eigen::Matrix4f T_map_odom_last = this->T_map_odom_;
  lock.unlock();

  pcl::PointCloud<PointType>::Ptr current_scan = std::make_shared<pcl::PointCloud<PointType>>();
  pcl::fromROSMsg(*pc_msg, *current_scan);

  if (this->vf_scan_use_) {
    this->vf_scan_.setInputCloud(current_scan);
    this->vf_scan_.filter(*current_scan);
  }

  // Use odometry and last correction to form an initial guess
  Eigen::Matrix4f T_odom_baselink = dlo::utils::poseMsgToEigen(odom_pose);
  Eigen::Matrix4f T_initial_guess = T_map_odom_last * T_odom_baselink;

  // Align scan to map
  this->gicp_.setInputSource(current_scan);
  pcl::PointCloud<PointType> aligned_scan;
  this->gicp_.align(aligned_scan, T_initial_guess);

  if (!this->gicp_.hasConverged()) {
    RCLCPP_WARN(this->get_logger(), "GICP did not converge.");
    return;
  }

  Eigen::Matrix4f T_map_baselink_new = this->gicp_.getFinalTransformation();
  
  // Update the map -> odom transform
  std::lock_guard<std::mutex> write_lock(this->odom_mutex_);
  this->T_map_odom_ = T_map_baselink_new * T_odom_baselink.inverse();
  this->publishTransform(pc_msg->header.stamp);
}

void LocalizationNode::publishTransform(const rclcpp::Time& stamp) {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = "map";
  t.child_frame_id = "odom";

  Eigen::Vector3f translation = this->T_map_odom_.block<3,1>(0,3);
  Eigen::Quaternionf rotation(this->T_map_odom_.block<3,3>(0,0));

  t.transform.translation.x = translation.x();
  t.transform.translation.y = translation.y();
  t.transform.translation.z = translation.z();
  t.transform.rotation.w = rotation.w();
  t.transform.rotation.x = rotation.x();
  t.transform.rotation.y = rotation.y();
  t.transform.rotation.z = rotation.z();

  this->tf_broadcaster_->sendTransform(t);
}

} // namespace dlo