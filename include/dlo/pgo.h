#ifndef PGO_H
#define PGO_H

#include "dlo.h"
#include <math.h>
#include <vector>
#include <queue>
#include <optional>

// ROS 2 Core Headers
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// TF2 Headers
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// PCL Headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

// GTSAM Headers
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.hh>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>

// Custom Headers
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "scancontext/Scancontext.h"

// Define PointType
typedef pcl::PointXYZI PointType;

// Define a 6-DOF pose structure
struct Pose6D {
    double x, y, z, roll, pitch, yaw;
    int seq; // sequence number for keyframe ID
};

class LaserPosegraphOptimization : public rclcpp::Node
{
public:
    LaserPosegraphOptimization();
    ~LaserPosegraphOptimization();

private:
    // ROS 2 Threads
    std::thread posegraph_thread_;
    std::thread lc_detection_thread_;
    std::thread icp_calculation_thread_;
    std::thread isam_update_thread_;
    std::thread viz_map_thread_;

    // ROS 2 Publishers and Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudFullRes;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftPGO;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathAftPGO;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapAftPGO;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pubKeyFramesId;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocalRegisted;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocal;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopSubmapLocal;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Buffers and Mutexes
    std::queue<nav_msgs::msg::Odometry::ConstSharedPtr> odometryBuf;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> fullResBuf;
    std::queue<std::pair<int, int>> scLoopICPBuf;
    std::mutex mBuf;
    std::mutex mKF;
    std::mutex mtxICP;
    std::mutex mtxPosegraph;
    std::mutex mtxRecentPose;

    // Parameters
    double keyframeMeterGap;
    double keyframeDegGap, keyframeRadGap;
    double scDistThres, scMaximumRadius;
    double historyKeyframeSearchRadius;
    double historyKeyframeSearchTimeDiff;
    int historyKeyframeSearchNum;
    double loopClosureFrequency;
    int graphUpdateTimes;
    double graphUpdateFrequency;
    double loopNoiseScore;
    double vizmapFrequency;
    double vizPathFrequency;
    double loopFitnessScoreThreshold;
    double mapVizFilterSize;
    std::string save_directory;

    // State variables
    double translationAccumulated = 1000000.0;
    double rotaionAccumulated = 1000000.0;
    bool isNowKeyFrame = false;
    Pose6D odom_pose_prev{0.0, 0.0, 0.0, 0.0, 0.0, 0};
    Pose6D odom_pose_curr{0.0, 0.0, 0.0, 0.0, 0.0, 0};
    double timeLaserOdometry = 0.0;
    double timeLaser = 0.0;

    pcl::PointCloud<PointType>::Ptr laserCloudFullRes{new pcl::PointCloud<PointType>()};
    pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO{new pcl::PointCloud<PointType>()};

    std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds;
    std::vector<Pose6D> keyframePoses;
    std::vector<Pose6D> keyframePosesUpdated;
    std::vector<double> keyframeTimes;
    int recentIdxUpdated = 0;
    
    std::map<int, int> loopIndexContainer;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeHistoryKeyPoses{new pcl::KdTreeFLANN<pcl::PointXYZ>()};

    // GTSAM
    gtsam::NonlinearFactorGraph gtSAMgraph;
    bool gtSAMgraphMade = false;
    gtsam::Values initialEstimate;
    gtsam::ISAM2 *isam;
    gtsam::Values isamCurrentEstimate;

    gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
    gtsam::noiseModel::Diagonal::shared_ptr odomNoise;
    gtsam::noiseModel::Base::shared_ptr robustLoopNoise;

    // Filters
    pcl::VoxelGrid<PointType> downSizeFilterScancontext;
    SCManager scManager;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::PointCloud<PointType>::Ptr laserCloudMapPGO{new pcl::PointCloud<PointType>()};
    pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
    bool laserCloudMapPGORedraw = true;

    // File I/O
    std::string pgKITTIformat, pgScansDirectory;
    std::string odomKITTIformat;
    std::fstream pgTimeSaveStream;

    // Function Declarations

    // Initialization
    void declare_parameters();
    void initNoises();

    // Callbacks
    void laserOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr _laserOdometry);
    void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr _laserCloudFullRes);

    // Main Processing Threads
    void process_pg();
    void process_lcd();
    void process_icp();
    void process_isam();
    void process_viz_map();

    // GTSAM & Optimization
    void runISAM2opt();
    void updatePoses();

    // Loop Closure
    bool detectLoopClosureDistance(int* loopKeyCur, int* loopKeyPre);
    void performRSLoopClosure();
    gtsam::Pose3 doICPVirtualRelative(int _loop_kf_idx, int _curr_kf_idx);
    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum);

    // Visualization
    void pubPath();
    void pubMap();
    void visualizeLoopClosure();

    // Helper Functions
    std::string padZeros(int val, int num_digits);
    Pose6D getOdom(nav_msgs::msg::Odometry::ConstSharedPtr _odom);
    Pose6D diffTransformation(const Pose6D& _p1, const Pose6D& _p2);
    pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr& cloudIn, const Pose6D& tf);
    gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p);
    Eigen::Affine3f Pose6dToAffine3f(Pose6D pose);
    pcl::PointCloud<pcl::PointXYZ>::Ptr vector2pc(const std::vector<Pose6D> vectorPose6d);
    
    // File I/O Helpers
    void saveOdometryVerticesKITTIformat(std::string _filename);
    void saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename);
};

#endif // PGO_H
