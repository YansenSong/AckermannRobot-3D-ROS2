/**
 * hdl_localization — ROS 2 NDT-based LiDAR localization node
 *
 * Converts ROS 1 hdl_localization nodelet → ROS 2 composable Node.
 *
 * Core algorithm:
 *   1. Load a pre-built 3D point cloud map (PCD) via globalmap_server callback
 *   2. For each incoming LiDAR scan:
 *      a. Transform to odom_child_frame using TF
 *      b. Downsample
 *      c. UKF predict (with optional IMU)
 *      d. NDT_OMP match against the global map → correct UKF
 *   3. Publish the estimated pose as map→odom TF + /odom topic
 *
 * Dependencies: ndt_omp, PCL, Eigen, tf2, rclcpp
 */

#include <mutex>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <pclomp/ndt_omp.h>

#include <hdl_localization/pose_estimator.hpp>

namespace hdl_localization {

class HdlLocalizationNode : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  HdlLocalizationNode()
  : Node("hdl_localization")
  {
    // ---- Parameters ----
    this->declare_parameter<std::string>("globalmap_pcd", "");
    this->declare_parameter<double>("downsample_resolution", 0.1);
    this->declare_parameter<std::string>("reg_method", "NDT_OMP");
    this->declare_parameter<std::string>("ndt_neighbor_search_method", "DIRECT7");
    this->declare_parameter<double>("ndt_neighbor_search_radius", 2.0);
    this->declare_parameter<double>("ndt_resolution", 1.0);
    this->declare_parameter<bool>("use_imu", true);
    this->declare_parameter<bool>("invert_acc", false);
    this->declare_parameter<bool>("invert_gyro", false);
    this->declare_parameter<std::string>("odom_child_frame_id", "lidar3d_link");
    this->declare_parameter<std::string>("robot_odom_frame_id", "odom");
    this->declare_parameter<bool>("specify_init_pose", true);
    this->declare_parameter<double>("init_pos_x", 0.0);
    this->declare_parameter<double>("init_pos_y", 0.0);
    this->declare_parameter<double>("init_pos_z", 0.0);
    this->declare_parameter<double>("init_ori_w", 1.0);
    this->declare_parameter<double>("init_ori_x", 0.0);
    this->declare_parameter<double>("init_ori_y", 0.0);
    this->declare_parameter<double>("init_ori_z", 0.0);
    this->declare_parameter<double>("cool_time_duration", 2.0);

    robot_odom_frame_id_ = this->get_parameter("robot_odom_frame_id").as_string();
    odom_child_frame_id_ = this->get_parameter("odom_child_frame_id").as_string();
    use_imu_ = this->get_parameter("use_imu").as_bool();
    invert_acc_ = this->get_parameter("invert_acc").as_bool();
    invert_gyro_ = this->get_parameter("invert_gyro").as_bool();

    // ---- TF ----
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // ---- Subscribers ----
    points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points_raw", rclcpp::SensorDataQoS(),
      std::bind(&HdlLocalizationNode::points_callback, this, std::placeholders::_1));

    if (use_imu_) {
      RCLCPP_INFO(this->get_logger(), "Enabling IMU-based prediction");
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_raw", rclcpp::SensorDataQoS(),
        std::bind(&HdlLocalizationNode::imu_callback, this, std::placeholders::_1));
    }

    globalmap_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/globalmap", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&HdlLocalizationNode::globalmap_callback, this, std::placeholders::_1));

    initialpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(10),
      std::bind(&HdlLocalizationNode::initialpose_callback, this, std::placeholders::_1));

    // ---- Publishers ----
    pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));
    aligned_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_points", rclcpp::QoS(10));

    // ---- Initialize scan matching ----
    initialize_params();

    RCLCPP_INFO(this->get_logger(),
      "hdl_localization ready. odom_child_frame: %s, robot_odom_frame: %s",
      odom_child_frame_id_.c_str(), robot_odom_frame_id_.c_str());
  }

private:
  /**
   * @brief Create registration method (NDT_OMP only, no CUDA dependency)
   */
  pcl::Registration<PointT, PointT>::Ptr create_registration() {
    std::string reg_method = this->get_parameter("reg_method").as_string();
    std::string ndt_search = this->get_parameter("ndt_neighbor_search_method").as_string();
    double ndt_radius = this->get_parameter("ndt_neighbor_search_radius").as_double();
    double ndt_res = this->get_parameter("ndt_resolution").as_double();

    if (reg_method == "NDT_OMP") {
      RCLCPP_INFO(this->get_logger(), "Using NDT_OMP registration");
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(
        new pclomp::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(0.01);
      ndt->setResolution(ndt_res);

      if (ndt_search == "DIRECT1") {
        RCLCPP_INFO(this->get_logger(), "  search_method: DIRECT1");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else if (ndt_search == "DIRECT7") {
        RCLCPP_INFO(this->get_logger(), "  search_method: DIRECT7");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      } else {
        RCLCPP_INFO(this->get_logger(), "  search_method: KDTREE");
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      }
      return ndt;
    }

    RCLCPP_ERROR(this->get_logger(), "Unknown registration method: %s. Only NDT_OMP is supported.",
                 reg_method.c_str());
    return nullptr;
  }

  void initialize_params() {
    // Downsample filter
    double downsample_res = this->get_parameter("downsample_resolution").as_double();
    auto voxelgrid = std::make_shared<pcl::VoxelGrid<PointT>>();
    voxelgrid->setLeafSize(downsample_res, downsample_res, downsample_res);
    downsample_filter_ = voxelgrid;

    // Registration
    registration_ = create_registration();
    if (!registration_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to create registration method");
      return;
    }

    // Pose estimator
    if (this->get_parameter("specify_init_pose").as_bool()) {
      RCLCPP_INFO(this->get_logger(), "Initializing pose estimator with specified parameters");
      pose_estimator_.reset(new PoseEstimator(
        registration_,
        Eigen::Vector3f(
          this->get_parameter("init_pos_x").as_double(),
          this->get_parameter("init_pos_y").as_double(),
          this->get_parameter("init_pos_z").as_double()),
        Eigen::Quaternionf(
          this->get_parameter("init_ori_w").as_double(),
          this->get_parameter("init_ori_x").as_double(),
          this->get_parameter("init_ori_y").as_double(),
          this->get_parameter("init_ori_z").as_double()),
        this->get_parameter("cool_time_duration").as_double()));
    }
  }

  // ---- Callbacks ----

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex_);
    imu_data_.push_back(imu_msg);
  }

  void globalmap_callback(const sensor_msgs::msg::PointCloud2::SharedPtr points_msg) {
    RCLCPP_INFO(this->get_logger(), "Global map received: %ux%u points",
                points_msg->width, points_msg->height);
    auto cloud = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::fromROSMsg(*points_msg, *cloud);
    globalmap_ = cloud;
    registration_->setInputTarget(globalmap_);
  }

  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg) {
    RCLCPP_INFO(this->get_logger(), "Initial pose received");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;
    pose_estimator_.reset(new PoseEstimator(
      registration_,
      Eigen::Vector3f(p.x, p.y, p.z),
      Eigen::Quaternionf(q.w, q.x, q.y, q.z),
      this->get_parameter("cool_time_duration").as_double()));
  }

  void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr points_msg) {
    if (!globalmap_) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Global map has not been received yet");
      return;
    }

    const auto stamp = rclcpp::Time(points_msg->header.stamp);
    auto cloud = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::fromROSMsg(*points_msg, *cloud);

    if (cloud->empty()) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Cloud is empty");
      return;
    }

    // Transform point cloud into odom_child_frame (lidar3d_link)
    auto transformed = std::make_shared<pcl::PointCloud<PointT>>();
    if (!transformPointCloud(odom_child_frame_id_, stamp, cloud, transformed)) {
      return;
    }

    // Downsample
    auto filtered = downsample(transformed);

    // Predict + Correct
    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex_);
    if (!pose_estimator_) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Waiting for initial pose");
      return;
    }

    // UKF predict (with or without IMU)
    if (!use_imu_) {
      pose_estimator_->predict(stamp);
    } else {
      std::lock_guard<std::mutex> lock(imu_data_mutex_);
      auto imu_iter = imu_data_.begin();
      for (; imu_iter != imu_data_.end(); ++imu_iter) {
        rclcpp::Time imu_stamp = (*imu_iter)->header.stamp;
        if (stamp < imu_stamp) {
          break;
        }
        const auto& acc = (*imu_iter)->linear_acceleration;
        const auto& gyro = (*imu_iter)->angular_velocity;
        double acc_sign = invert_acc_ ? -1.0 : 1.0;
        double gyro_sign = invert_gyro_ ? -1.0 : 1.0;
        pose_estimator_->predict(
          imu_stamp,
          acc_sign * Eigen::Vector3f(acc.x, acc.y, acc.z),
          gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
      }
      imu_data_.erase(imu_data_.begin(), imu_iter);
    }

    // NDT correction
    auto aligned = pose_estimator_->correct(stamp, filtered);

    // Publish aligned cloud (for visualization)
    if (aligned_pub_->get_subscription_count() > 0) {
      auto aligned_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(*aligned, *aligned_msg);
      aligned_msg->header.stamp = points_msg->header.stamp;
      aligned_msg->header.frame_id = "map";
      aligned_pub_->publish(std::move(aligned_msg));
    }

    // Publish odometry and TF
    publish_odometry(stamp, pose_estimator_->matrix());
  }

  // ---- Utility functions ----

  /**
   * @brief Transform point cloud using TF
   */
  bool transformPointCloud(
      const std::string& target_frame,
      const rclcpp::Time& stamp,
      const pcl::PointCloud<PointT>::ConstPtr& cloud_in,
      pcl::PointCloud<PointT>::Ptr cloud_out)
  {
    // Convert PCL → ROS msg
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_in, cloud_msg);

    // Look up transform
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        target_frame, cloud_msg.header.frame_id, stamp,
        rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(), "TF transform failed: %s", ex.what());
      return false;
    }

    // Apply transform
    sensor_msgs::msg::PointCloud2 transformed_msg;
    tf2::doTransform(cloud_msg, transformed_msg, transform);

    // Convert back to PCL
    pcl::fromROSMsg(transformed_msg, *cloud_out);
    cloud_out->header = pcl_conversions::toPCL(transformed_msg.header);
    return true;
  }

  pcl::PointCloud<PointT>::ConstPtr downsample(
      const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if (!downsample_filter_) {
      return cloud;
    }
    auto filtered = std::make_shared<pcl::PointCloud<PointT>>();
    downsample_filter_->setInputCloud(cloud);
    downsample_filter_->filter(*filtered);
    filtered->header = cloud->header;
    return filtered;
  }

  /**
   * @brief Publish map→odom TF, odom→base_link TF, and /odom topic
   *
   * TF convention (matches existing project):
   *   map → odom : identity (NDT gives direct map-frame pose)
   *   odom → base_link : NDT localization result
   *
   * The /odom topic is consumed by nav2_plan_bridge to publish odom→base_link TF.
   */
  void publish_odometry(const rclcpp::Time& stamp, const Eigen::Matrix4f& pose) {
    // 1. map → odom identity (NDT pose is already in map frame)
    {
      geometry_msgs::msg::TransformStamped map_to_odom;
      map_to_odom.header.stamp = stamp;
      map_to_odom.header.frame_id = "map";
      map_to_odom.child_frame_id = robot_odom_frame_id_;
      map_to_odom.transform.translation.x = 0.0;
      map_to_odom.transform.translation.y = 0.0;
      map_to_odom.transform.translation.z = 0.0;
      map_to_odom.transform.rotation.x = 0.0;
      map_to_odom.transform.rotation.y = 0.0;
      map_to_odom.transform.rotation.z = 0.0;
      map_to_odom.transform.rotation.w = 1.0;
      tf_broadcaster_->sendTransform(map_to_odom);
    }

    // 2. odom → base_link from NDT pose
    {
      Eigen::Isometry3d pose_iso;
      pose_iso.matrix() = pose.cast<double>();
      geometry_msgs::msg::TransformStamped odom_to_base =
        tf2::eigenToTransform(pose_iso);
      odom_to_base.header.stamp = stamp;
      odom_to_base.header.frame_id = robot_odom_frame_id_;
      odom_to_base.child_frame_id = "base_link";
      tf_broadcaster_->sendTransform(odom_to_base);

      // 3. Publish /odom topic
      auto odom = nav_msgs::msg::Odometry();
      odom.header.stamp = stamp;
      odom.header.frame_id = robot_odom_frame_id_;
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = pose(0, 3);
      odom.pose.pose.position.y = pose(1, 3);
      odom.pose.pose.position.z = pose(2, 3);
      Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      odom.pose.pose.orientation.w = q.w();
      pose_pub_->publish(odom);
    }
  }

  // ---- Member variables ----

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Parameters
  std::string robot_odom_frame_id_;
  std::string odom_child_frame_id_;
  bool use_imu_;
  bool invert_acc_;
  bool invert_gyro_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;

  // IMU buffer
  std::mutex imu_data_mutex_;
  std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_data_;

  // Global map + registration
  pcl::PointCloud<PointT>::Ptr globalmap_;
  pcl::Filter<PointT>::Ptr downsample_filter_;
  pcl::Registration<PointT, PointT>::Ptr registration_;

  // Pose estimator
  std::mutex pose_estimator_mutex_;
  std::unique_ptr<PoseEstimator> pose_estimator_;
};

}  // namespace hdl_localization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hdl_localization::HdlLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
