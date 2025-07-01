#include "dlo/localization.h"
#include "dlo/utils.h"

dlo::LocalizationNode::LocalizationNode() : Node("dlo_localization_node") {
  
  RCLCPP_INFO(this->get_logger(), "Initializing DLO Localization Node");

  this->getParams();

  if (!this->initial_pose_use_) {
    RCLCPP_INFO(this->get_logger(), "Using initial pose from /initialpose topic");
    this->is_initialized_ = false;
  }
  else {
    RCLCPP_INFO(this->get_logger(), "Using provided initial pose");
    this->T_map_odom_.block(0,3,3,1) = this->initial_position_;
    this->T_map_odom_.block(0,0,3,3) = this->initial_orientation_.toRotationMatrix();
    this->is_initialized_ = true;
  }

  this->loadGlobalMap();

  this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 1, std::bind(&dlo::LocalizationNode::odomCallback, this, std::placeholders::_1));
  this->pc_sub_   = this->create_subscription<sensor_msgs::msg::PointCloud2>("filtered_scan", 1, std::bind(&dlo::LocalizationNode::pointcloudCallback, this, std::placeholders::_1)); 

}

// destructor
dlo::LocalizationNode::~LocalizationNode() {}

void dlo::LocalizationNode::start() {
  RCLCPP_INFO(this->get_logger(), "Starting DLO Localization Node");
}

void dlo::LocalizationNode::getParams() {
  this->declare_parameter<std::string>("map_path", "global_map.pcd");
  this->get_parameter("map_path", this->map_path_);

  this->declare_parameter<bool>("initial_pose_use", false);
  this->get_parameter("initial_pose_use", this->initial_pose_use_);

  double px, py, pz, qx, qy, qz, qw;
  this->declare_parameter<double>("dlo/localizationNode/initial_position/x", 0.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_position/y", 0.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_position/z", 0.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_orientation/w", 1.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_orientation/x", 0.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_orientation/y", 0.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_orientation/z", 0.0);
  this->get_parameter("dlo/localizationNode/initial_position/x", px);
  this->get_parameter("dlo/localizationNode/initial_position/y", py);
  this->get_parameter("dlo/localizationNode/initial_position/z", pz);
  this->get_parameter("dlo/localizationNode/initial_orientation/w", qw);
  this->get_parameter("dlo/localizationNode/initial_orientation/x", qx);
  this->get_parameter("dlo/localizationNode/initial_orientation/y", qy);
  this->get_parameter("dlo/localizationNode/initial_orientation/z", qz);
  this->initial_position_ = Eigen::Vector3f(px, py, pz);
  this->initial_orientation_ = Eigen::Quaternionf(qw, qx, qy, qz);
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

  this->current_scan_ = std::make_shared<pcl::PointCloud<PointType>>();
  pcl::fromROSMsg(*pc_msg, *this->current_scan_);

  // create the initialization guess based on the latest odom pose

  this->debug();

}

// Debug method to print map load status and node info
void dlo::LocalizationNode::debug() {
  std::cout << std::endl << "==== Direct LiDAR Localization ====" << std::endl;
  if (this->global_map_) {
    std::cout << "Global map path: " << this->map_path_ << std::endl;
    std::cout << "Global map points: " << this->global_map_->points.size() << std::endl;
    if (this->global_map_->points.size() > 0) {
      RCLCPP_INFO(this->get_logger(), "Map loaded successfully!");
    } else {
      std::cout << "Map pointer valid but contains 0 points!" << std::endl;
    }
  } else {
    std::cout << "Map not loaded!" << std::endl;
  }
}