#include "dlo/localization.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

dlo::LocalizationNode::LocalizationNode() : Node("dlo_localization_node") {

  RCLCPP_INFO(this->get_logger(), "Initializing DLO Localization Node");

  this->declare_parameter<std::string>("dlo/localizationNode/map_path", "global_map.pcd");
  this->declare_parameter<double>("dlo/localizationNode/submap_size", 50.0);
  this->declare_parameter<bool>("dlo/localizationNode/initial_pose_use", false);
  this->declare_parameter<double>("dlo/localizationNode/initial_position/x", 0.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_position/y", 0.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_position/z", 0.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_orientation/w", 1.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_orientation/x", 0.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_orientation/y", 0.0);
  this->declare_parameter<double>("dlo/localizationNode/initial_orientation/z", 0.0);

  this->declare_parameter<int>("dlo/odomNode/gicp/s2m/kCorrespondences", 20);
  this->declare_parameter<double>("dlo/odomNode/gicp/s2m/maxCorrespondenceDistance", 0.5);
  this->declare_parameter<int>("dlo/odomNode/gicp/s2m/maxIterations", 32);
  this->declare_parameter<double>("dlo/odomNode/gicp/s2m/transformationEpsilon", 0.01);
  this->declare_parameter<double>("dlo/odomNode/gicp/s2m/euclideanFitnessEpsilon", 0.01);
  this->declare_parameter<int>("dlo/odomNode/gicp/s2m/ransac/iterations", 5);
  this->declare_parameter<double>("dlo/odomNode/gicp/s2m/ransac/outlierRejectionThresh", 1.0);

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

  this->pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud", 1, std::bind(&dlo::LocalizationNode::pointcloudCallback, this, std::placeholders::_1));

  this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 1, std::bind(&dlo::LocalizationNode::odomCallback, this, std::placeholders::_1));

  this->initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1, std::bind(&dlo::LocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
  this->tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

  // load and publish the global map
  this->loadGlobalMap();

  // initialize the GICP
  this->setupGICP();
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
  std::string map_path;
  this->get_parameter("dlo/localizationNode/map_path", map_path);
  this->global_map_ = std::make_shared<pcl::PointCloud<PointType>>();

  if (pcl::io::loadPCDFile<PointType>(map_path, *this->global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load global map from %s", map_path.c_str());
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Global map loaded with %zu points", this->global_map_->points.size());
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
  this->gicp_.setSearchMethodSource(temp, true);
  this->gicp_.setSearchMethodTarget(temp, true);
}

void dlo::LocalizationNode::publishTransform(const rclcpp::Time& stamp) {
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = "map";
  transform_msg.child_frame_id = "odom_lidar";

  Eigen::Matrix4f T_map_odom = this->T_map_odom_;
  transform_msg.transform.translation.x = T_map_odom(0, 3);
  transform_msg.transform.translation.y = T_map_odom(1, 3);
  transform_msg.transform.translation.z = T_map_odom(2, 3);

  Eigen::Quaternionf q(T_map_odom.block<3, 3>(0, 0));
  transform_msg.transform.rotation.w = q.w();
  transform_msg.transform.rotation.x = q.x();
  transform_msg.transform.rotation.y = q.y();
  transform_msg.transform.rotation.z = q.z();

  this->tf_broadcaster_->sendTransform(transform_msg);
}

void dlo::LocalizationNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  std::lock_guard<std::mutex> lock(this->icp_mutex_);

  // Extract the position and orientation from the received message
  Eigen::Translation3f translation(
      static_cast<float>(odom_msg->pose.pose.position.x),
      static_cast<float>(odom_msg->pose.pose.position.y),
      static_cast<float>(odom_msg->pose.pose.position.z)
  );
  Eigen::Quaternionf rotation(
      static_cast<float>(odom_msg->pose.pose.orientation.w),
      static_cast<float>(odom_msg->pose.pose.orientation.x),
      static_cast<float>(odom_msg->pose.pose.orientation.y),
      static_cast<float>(odom_msg->pose.pose.orientation.z)
  );

  this->T_odom_baselink_ = (translation * rotation).matrix();
}

// The initial pose of the robot base link frame w.r.t to the map frame
void dlo::LocalizationNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(this->icp_mutex_);

  // Extract the position and orientation from the received message
  Eigen::Translation3f translation(
      static_cast<float>(msg->pose.pose.position.x),
      static_cast<float>(msg->pose.pose.position.y),
      static_cast<float>(msg->pose.pose.position.z)
  );
  Eigen::Quaternionf rotation(
      static_cast<float>(msg->pose.pose.orientation.w),
      static_cast<float>(msg->pose.pose.orientation.x),
      static_cast<float>(msg->pose.pose.orientation.y),
      static_cast<float>(msg->pose.pose.orientation.z)
  );

  Eigen::Matrix4f T_map_baselink = (translation * rotation).matrix();
  this->T_map_odom_ = T_map_baselink * this->T_odom_baselink_.inverse();
  this->is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "Initial pose set: Position [%f, %f, %f], Orientation [%f, %f, %f, %f]",
              translation.x(), translation.y(), translation.z(),
              rotation.w(), rotation.x(), rotation.y(), rotation.z());
}

// Utilize DLO's keyframe point cloud to help localize the robot in the global map to avoid drifting
void dlo::LocalizationNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg) {
  rclcpp::Time scan_stamp = pc_msg->header.stamp;

  if (!this->is_initialized_) {
    RCLCPP_WARN(this->get_logger(), "Localization node not initialized, waiting for initial pose");
    return;
  }

  // Lock the mutex to safely access the latest odom pose and transformation
  std::unique_lock<std::mutex> lock(this->icp_mutex_);
  Eigen::Matrix4f T_map_odom_last = this->T_map_odom_;
  lock.unlock();

  // Extract Local Map from Global Map
  pcl::PointCloud<PointType>::Ptr local_map(new pcl::PointCloud<PointType>);
  double submap_size = this->get_parameter("dlo/localizationNode/submap_size").as_double();

  pcl::CropBox<PointType> crop_box_filter;
  crop_box_filter.setInputCloud(this->global_map_);
  Eigen::Vector4f min_pt, max_pt;
  min_pt << -submap_size/2.0, -submap_size/2.0, -5.0, 1.0;
  max_pt << submap_size/2.0, submap_size/2.0, 5.0, 1.0;

  // Transform the bounding box to be centered at the robot's current pose
  Eigen::Affine3f transform(T_map_odom_last);
  crop_box_filter.setMin(min_pt);
  crop_box_filter.setMax(max_pt);
  crop_box_filter.setTransform(transform);

  crop_box_filter.filter(*local_map);

  // Set local map as GICP target
  this->gicp_.setInputTarget(local_map);
  this->gicp_.calculateTargetCovariances();

  // Set input source for GICP
  pcl::PointCloud<PointType>::Ptr current_scan = std::make_shared<pcl::PointCloud<PointType>>();
  pcl::fromROSMsg(*pc_msg, *current_scan);
  this->gicp_.setInputSource(current_scan);

  pcl::PointCloud<PointType>::Ptr aligned = std::make_shared<pcl::PointCloud<PointType>>();
  this->gicp_.align(*aligned, T_map_odom_last);

  // Compute the final transformation
  Eigen::Matrix4f T_map_odom_new = this->gicp_.getFinalTransformation();

  lock.lock();
  this->T_map_odom_ = T_map_odom_new;
  lock.unlock();

  // Publish the transformation
  this->publishTransform(scan_stamp);

  // this->debug();
}

// Debug method to print map load status and node info
void dlo::LocalizationNode::debug() {
  std::stringstream ss;
  std::lock_guard<std::mutex> lock(this->icp_mutex_);

  Eigen::Vector3f position = this->T_map_odom_.block<3, 1>(0, 3);
  Eigen::Quaternionf rotation(this->T_map_odom_.block<3, 3>(0, 0));

  ss << std::endl << "==== Direct LiDAR Localization ====" << std::endl;
  if (this->global_map_) {
    ss << "Global map points: " << this->global_map_->points.size() << std::endl;
    if (this->global_map_->points.size() > 0) {
      ss << "Mat loaded successfully!" << std::endl;
    } else {
      ss << "Map pointer valid but contains 0 points!" << std::endl;
    }
  } else {
    ss << "Map not loaded!" << std::endl;
  }
  ss << "Current Map->Odom Pose: " << std::endl;
  ss << "Position: [" << position.x() << ", " << position.y() << ", " << position.z() << "]" << std::endl;
  ss << "Orientation: [" << rotation.w() << ", " << rotation.x() << ", " << rotation.y() << " , " << rotation.z() << "]" << std::endl;

  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}
