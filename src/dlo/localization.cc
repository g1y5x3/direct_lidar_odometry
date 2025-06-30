#include "dlo/localization.h"
#include "dlo/utils.h"

dlo::LocalizationNode::LocalizationNode() : Node("dlo_localization_node") {
  
  this->getParams();

  this->loadGlobalMap();

  // CREATE A SUBSCRIBER FOR THE FILTERED SCAN

}

// destructor
dlo::LocalizationNode::~LocalizationNode() {}

void dlo::LocalizationNode::start() {
  RCLCPP_INFO(this->get_logger(), "Starting DLO Localization Node");
}

void dlo::LocalizationNode::getParams() {
  // REPLACE WITH dlo::declare_param LATER
  this->declare_parameter<std::string>("map_path", "global_map.pcd");
  this->declare_parameter<bool>("crop_use", true);
  this->declare_parameter<double>("crop_size", 1.0);
  this->declare_parameter<bool>("vf_scan_use", true);
  this->declare_parameter<double>("vf_scan_res", 0.05);
  this->declare_parameter<int>("gicp_min_num_points", 100);

  this->get_parameter("map_path", this->map_path_);
  this->get_parameter("crop_use", this->crop_use_);
  this->get_parameter("crop_size", this->crop_size_);
  this->get_parameter("vf_scan_use", this->vf_scan_use_);
  this->get_parameter("vf_scan_res", this->vf_scan_res_);
}

void dlo::LocalizationNode::loadGlobalMap() {
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local();
  this->map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", qos);

  this->global_map_ = std::make_shared<pcl::PointCloud<PointType>>();
  if (pcl::io::loadPCDFile<PointType>(this->map_path_, *this->global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load global map from %s", this->map_path_.c_str());
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Global map loaded with %zu points", this->global_map_->points.size());

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*this->global_map_, map_msg);

  map_msg.header.frame_id = "map";
  map_msg.header.stamp = this->now();
  this->map_pub_->publish(map_msg);
  RCLCPP_INFO(this->get_logger(), "Global map published");

  // Initialize map_to_odom transform
  this->T_map_odom_ = Eigen::Matrix4f::Identity();
}

void dlo::LocalizationNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(this->odom_mutex_);
  this->latest_odom_pose_ = msg->pose.pose;
}


void dlo::LocalizationNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg) {
  // ADD INITIALIZATION CHECK

  // update the current odom pose
  std::unique_lock<std::mutex> lock(this->odom_mutex_);
  if (!this->latest_odom_pose_) {
    RCLCPP_WARN(this->get_logger(), "No latest odom pose available, skipping pointcloud processing");
    return;
  }
  geometry_msgs::msg::Pose current_pose = *this->latest_odom_pose_;
  lock.unlock();

  // filter the incoming point cloud if necessary
  this->current_scan_ = std::make_shared<pcl::PointCloud<PointType>>();
  pcl::fromROSMsg(*pc_msg, *this->current_scan_);

  // create the initialization guess based on the latest odom pose

}