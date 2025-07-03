#include "dlo/localization.h"

dlo::LocalizationNode::LocalizationNode() : Node("dlo_localization_node") {

  RCLCPP_INFO(this->get_logger(), "Initializing DLO Localization Node");

  // Parameters declaration
  this->declare_parameter<bool>("dlo/localizationNode/initial_pose_use", true);
  this->declare_parameter<double>("dlo/localizationNode/initial_position/x", 0.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_position/y", 0.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_position/z", 0.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_orientation/w", 1.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_orientation/x", 0.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_orientation/y", 0.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_orientation/z", 0.0);

  this->declare_parameter<std::string>("dlo/localizationNode/map_path", "global_map.pcd");

  this->declare_parameter<int>("dlo/odomNode/gicp/s2m/kCorrespondences", 20);
  this->declare_parameter<double>("dlo/odomNode/gicp/s2m/maxCorrespondenceDistance", std::sqrt(std::numeric_limits<double>::max()));
  this->declare_parameter<int>("dlo/odomNode/gicp/s2m/maxIterations", 64);
  this->declare_parameter<double>("dlo/odomNode/gicp/s2m/transformationEpsilon", 0.0005);
  this->declare_parameter<double>("dlo/odomNode/gicp/s2m/euclideanFitnessEpsilon", -std::numeric_limits<double>::max());
  this->declare_parameter<int>("dlo/odomNode/gicp/s2m/ransac/iterations", 0);
  this->declare_parameter<double>("dlo/odomNode/gicp/s2m/ransac/outlierRejectionThresh", 0.05);

  // read map -> odom initialization 
  this->getinitParams();
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

  // load and publish the global map
  this->loadGlobalMap();

  // initialize the GICP
  this->setupGICP();

  // this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 1, std::bind(&dlo::LocalizationNode::odomCallback, this, std::placeholders::_1));
  // this->pc_sub_   = this->create_subscription<sensor_msgs::msg::PointCloud2>("filtered_scan", 1, std::bind(&dlo::LocalizationNode::pointcloudCallback, this, std::placeholders::_1)); 

}

// destructor
dlo::LocalizationNode::~LocalizationNode() {}

void dlo::LocalizationNode::start() {
  RCLCPP_INFO(this->get_logger(), "Starting DLO Localization Node");
}

void dlo::LocalizationNode::getinitParams() {
  this->get_parameter("dlo/localizationNode/initial_pose_use", this->initial_pose_use_);

  double px, py, pz, qx, qy, qz, qw;
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
  // initialization
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local();
  this->map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", qos);

  // load the point cloud map
  std::string map_path; 
  this->get_parameter("dlo/localizationNode/map_path", map_path);
  this->global_map_ = std::make_shared<pcl::PointCloud<PointType>>();

  if (pcl::io::loadPCDFile<PointType>(map_path, *this->global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load global map from %s", map_path.c_str());
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Global map loaded with %zu points", this->global_map_->points.size());

  // publish the map
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*this->global_map_, map_msg);
  map_msg.header.frame_id = "map";
  map_msg.header.stamp = this->now();
  this->map_pub_->publish(map_msg);
  RCLCPP_INFO(this->get_logger(), "Global map published");

}

void dlo::LocalizationNode::setupGICP() {
  int kCorrespondences, maxIterations, ransacIterations;
  double maxCorrespondenceDistance, transformationEpsilon, euclideanFitnessEpsilon, ransacOutlierRejectionThresh;

  this->get_parameter("dlo/odomNode/gicp/s2m/kCorrespondences", kCorrespondences);
  this->get_parameter("dlo/odomNode/gicp/s2m/maxCorrespondenceDistance", maxCorrespondenceDistance);
  this->get_parameter("dlo/odomNode/gicp/s2m/maxIterations", maxIterations);
  this->get_parameter("dlo/odomNode/gicp/s2m/transformationEpsilon", transformationEpsilon);
  this->get_parameter("dlo/odomNode/gicp/s2m/euclideanFitnessEpsilon", euclideanFitnessEpsilon);
  this->get_parameter("dlo/odomNode/gicp/s2m/ransac/iterations", ransacIterations);
  this->get_parameter("dlo/odomNode/gicp/s2m/ransac/outlierRejectionThresh", ransacOutlierRejectionThresh);

  // Initialize GICP parameters
  this->gicp_.setCorrespondenceRandomness(kCorrespondences);
  this->gicp_.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
  this->gicp_.setMaximumIterations(maxIterations);
  this->gicp_.setTransformationEpsilon(transformationEpsilon);
  this->gicp_.setEuclideanFitnessEpsilon(euclideanFitnessEpsilon);
  this->gicp_.setRANSACIterations(ransacIterations);
  this->gicp_.setRANSACOutlierRejectionThreshold(ransacOutlierRejectionThresh);

  pcl::Registration<PointType, PointType>::KdTreeReciprocalPtr temp;
  this->gicp_.setSearchMethodSource(temp);
  this->gicp_.setSearchMethodTarget(temp);

  this->gicp_.setInputTarget(this->global_map_);
  this->gicp_.calculateTargetCovariances();

  RCLCPP_INFO(this->get_logger(), "GICP initialization completed!");
}

void dlo::LocalizationNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(this->odom_mutex_);
}

void dlo::LocalizationNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(this->odom_mutex_);
  this->latest_odom_pose_ = msg->pose.pose;
}

void dlo::LocalizationNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg) {
  if (!this->is_initialized_) {
    RCLCPP_WARN(this->get_logger(), "Localization node not initialized, waiting for initial pose");
    return;
  }

  std::unique_lock<std::mutex> lock(this->odom_mutex_);
  if (!this->latest_odom_pose_) {
    RCLCPP_WARN(this->get_logger(), "No latest odom pose available, skipping pointcloud processing");
    return;
  }
  geometry_msgs::msg::Pose current_pose = *this->latest_odom_pose_;
  Eigen::Matrix4f T_map_odom_last = this->T_map_odom_;
  lock.unlock();

  // Convert ROS messages to PCL and Eigen formats
  pcl::PointCloud<PointType>::Ptr current_scan = std::make_shared<pcl::PointCloud<PointType>>();
  pcl::fromROSMsg(*pc_msg, *current_scan_);
  Eigen::Matrix4f T_odom_base = dlo::poseMsgToEigen(current_pose);

  // Create the initial guess for GICP
  Eigen::Matrix4f T_inital_guess = T_map_odom_last * T_odom_base_;
  
  // Align the current scan to the glocal map

  this->debug();

}

// Debug method to print map load status and node info
void dlo::LocalizationNode::debug() {
  std::cout << std::endl << "==== Direct LiDAR Localization ====" << std::endl;
  if (this->global_map_) {
    std::cout << "Global map points: " << this->global_map_->points.size() << std::endl;
    if (this->global_map_->points.size() > 0) {
      std::cout << "Mat loaded successfully!" << std::endl;
    } else {
      std::cout << "Map pointer valid but contains 0 points!" << std::endl;
    }
  } else {
    std::cout << "Map not loaded!" << std::endl;
  }
}