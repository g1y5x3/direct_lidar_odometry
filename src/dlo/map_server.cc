#include "dlo/map_server.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

dlo::MapServer::MapServer() : Node("dlo_map_server_node")
{
  RCLCPP_INFO(this->get_logger(), "Initializing DLO Map Server Node");

  // Declare parameters
  this->declare_parameter<std::string>("map_path", "global_map.pcd");
  this->declare_parameter<double>("map_leaf_size", 0.2);
  this->declare_parameter<double>("ransac_distance_threshold", 0.2);

  // Setup publisher with transient local QoS to latch the map
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local();
  this->map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", qos);

  this->global_map_ = std::make_shared<pcl::PointCloud<PointType>>();
}

dlo::MapServer::~MapServer() {}

void dlo::MapServer::start()
{
  this->setupMap();
  this->map_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200), // 5Hz
    std::bind(&dlo::MapServer::publishMapCallback, this)
  );
  RCLCPP_INFO(this->get_logger(), "Map server started. Publishing map at 5Hz.");
}

void dlo::MapServer::setupMap()
{
  // Load
  std::string map_path = this->get_parameter("map_path").as_string();
  pcl::PointCloud<PointType>::Ptr raw_map = std::make_shared<pcl::PointCloud<PointType>>();
  if (pcl::io::loadPCDFile<PointType>(map_path, *raw_map) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load global map from %s", map_path.c_str());
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Raw map loaded with %zu points from %s", raw_map->points.size(), map_path.c_str());

  // Filter
  double map_leaf_size = this->get_parameter("map_leaf_size").as_double();
  if (map_leaf_size > 0.0) {
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setLeafSize(map_leaf_size, map_leaf_size, map_leaf_size);
    voxel_grid.setInputCloud(raw_map);
    voxel_grid.filter(*this->global_map_);
    RCLCPP_INFO(this->get_logger(), "Filtered map to %zu points", this->global_map_->points.size());
  } else {
    this->global_map_ = raw_map;
    RCLCPP_INFO(this->get_logger(), "No filtering applied to map.");
  }
  
  // Pre-process intensities using RANSAC ground segmentation
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<PointType> seg;
  
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(this->get_parameter("ransac_distance_threshold").as_double());
  seg.setInputCloud(this->global_map_);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
  {
    RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset. All points will be marked as non-ground.");
    for (std::size_t i = 0; i < this->global_map_->points.size(); ++i) {
      this->global_map_->points[i].intensity = 1.0; // Non-Ground
    }
  }
  else
  {
    // Assume all points are non-ground initially
    for (std::size_t i = 0; i < this->global_map_->points.size(); ++i) {
      this->global_map_->points[i].intensity = 1.0; // Non-Ground
    }
    
    // Mark RANSAC inliers as ground
    for (std::size_t i = 0; i < inliers->indices.size(); ++i) {
      this->global_map_->points[inliers->indices[i]].intensity = 0.0; // Ground
    }
    RCLCPP_INFO(this->get_logger(), "Segmented ground plane with %zu points.", inliers->indices.size());
  }
}
void dlo::MapServer::publishMapCallback()
{
  // Publish
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*this->global_map_, map_msg);
  map_msg.header.frame_id = "map";
  map_msg.header.stamp = this->now();
  this->map_pub_->publish(map_msg);
}
