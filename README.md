# Direct LiDAR Odometry: <br> Fast Localization with Dense Point Clouds

## Note

This is a port after 

[Cardinal-Space-Mining](https://github.com/Cardinal-Space-Mining/direct_lidar_odometry) initial effort to move it to ROS 2

and

[tu-darmstadt-ros-pkg](https://github.com/tu-darmstadt-ros-pkg/direct_lidar_odometry) do have map pcd properly published

Tested on ROS 2 Humble with Ubuntu 22.04

### Services
To save DLO's generated map into `.pcd` format, call the following service:

```sh
ros2 service call /dlo_map/save_pcd direct_lidar_odometry/srv/SavePCD "{'leaf_size': 0.25, 'save_path': 'map'}"
```
To save the trajectory in KITTI format, call the following service:

__Not supported yet!__
<!-- ```sh
rosservice call /robot/dlo_odom/save_traj SAVE_PATH
``` -->