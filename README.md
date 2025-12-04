# Direct LiDAR Odometry for ROS 2

## Overview

This package is a ROS 2 port of the `direct_lidar_odometry` package, providing fast localization with dense point clouds.

This version includes significant enhancements to the `dlo_map_server_node` for robust, real-time ground segmentation and obstacle cloud generation, making it suitable for navigation tasks on varied terrain.

> **Note**: This is a port based on the initial ROS 2 efforts from [Cardinal-Space-Mining](https://github.com/Cardinal-Space-Mining/direct_lidar_odometry) and includes map publishing features from [tu-darmstadt-ros-pkg](https://github.com/tu-darmstadt-ros-pkg/direct_lidar_odometry). It has been tested on ROS 2 Humble with Ubuntu 22.04.

## Dependencies

This package depends on the following ROS 2 packages:
- `sensor_msgs`
- `geometry_msgs`
- `nav_msgs`
- `std_msgs`
- `rclcpp`
- `pcl_conversions`
- `tf2_ros`
- `tf2_sensor_msgs`

And the following system dependencies:
- `PCL`
- `Eigen`

Ensure you have all dependencies installed in your workspace or system. For example:
```bash
sudo apt-get install ros-humble-pcl-conversions ros-humble-tf2-sensor-msgs
```

## Building

To build the package, clone it into your ROS 2 workspace and build with `colcon`:
```bash
cd /path/to/your/ros2_ws/src
# git clone ...
cd ..
colcon build --packages-select direct_lidar_odometry
```

## Nodes

### `dlo_odom_node`
The primary node that performs LiDAR odometry by registering incoming point clouds against a submap of previous keyframes.

### `dlo_map_server_node`
This node serves two main purposes:
1.  **Global Map Publisher**: It loads a pre-existing `.pcd` map, applies a voxel grid filter, and publishes it as a static `/global_cloud`.
2.  **Obstacle Cloud Generator**: It subscribes to live point clouds, performs robust ground plane segmentation, and publishes the resulting non-ground points as an `/obstacle_cloud` for use in navigation and planning.

**Ground Segmentation Method:**
The node uses an `ApproximateProgressiveMorphologicalFilter` (APMF) to distinguish ground from non-ground points. The point cloud is first transformed into the robot's `base_link` frame to ensure segmentation is consistent regardless of the robot's orientation.

## Usage

To use the full functionality, you will typically run both the odometry and the map server nodes. Example launch files are provided.

```bash
ros2 launch direct_lidar_odometry dlo.launch.py
ros2 launch direct_lidar_odometry dlo.mapping.launch.py
```

### Important Configuration

For the ground segmentation to function correctly, the odometry system must be gravity-aligned. This ensures the robot's `base_link` frame is correctly oriented with respect to the world, which is crucial for the point cloud transformation before segmentation.

In your configuration file (e.g., `cfg/dlo.yaml`), ensure `gravityAlign` is set to `true`:
```yaml
dlo:
  gravityAlign: true
```

## Services

### Save Map
To save the generated map from the `dlo_map_node` into a `.pcd` file, call the following service:
```bash
ros2 service call /dlo_map/save_pcd direct_lidar_odometry/srv/SavePCD "{'leaf_size': 0.25, 'save_path': 'map'}"
```
This will save a file named `map.pcd` in the directory where the node was launched.
