/**
 * globalmap_server_node — ROS 2 port of globalmap_server_nodelet
 *
 * Loads a pre-built 3D point cloud map (PCD file) and publishes it as a
 * latched /globalmap topic for hdl_localization to consume.
 *
 * Supports optional UTM-to-local coordinate offset and downsampling.
 */

#include <fstream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

class GlobalmapServerNode : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  GlobalmapServerNode()
  : Node("globalmap_server")
  {
    this->declare_parameter<std::string>("globalmap_pcd", "");
    this->declare_parameter<double>("downsample_resolution", 0.2);
    this->declare_parameter<bool>("convert_utm_to_local", false);

    // Latched publisher
    globalmap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/globalmap", rclcpp::QoS(1).transient_local().reliable());

    load_and_publish();

    // Timer to republish (ensures late subscribers get the map)
    repub_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      [this]() { load_and_publish(); });
  }

private:
  void load_and_publish() {
    std::string pcd_path = this->get_parameter("globalmap_pcd").as_string();
    if (pcd_path.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
        "globalmap_pcd parameter not set. Waiting...");
      return;
    }

    auto cloud = std::make_shared<pcl::PointCloud<PointT>>();
    if (pcl::io::loadPCDFile(pcd_path, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load PCD: %s", pcd_path.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(),
      "Loaded PCD: %s (%zu points)", pcd_path.c_str(), cloud->size());

    // UTM offset
    if (this->get_parameter("convert_utm_to_local").as_bool()) {
      std::string utm_path = pcd_path + ".utm";
      std::ifstream utm_file(utm_path);
      if (utm_file.is_open()) {
        double utm_easting, utm_northing, altitude;
        utm_file >> utm_easting >> utm_northing >> altitude;
        for (auto& pt : cloud->points) {
          pt.getVector3fMap() -= Eigen::Vector3f(utm_easting, utm_northing, altitude);
        }
        RCLCPP_INFO(this->get_logger(),
          "Applied UTM offset: (%f, %f, %f)", utm_easting, utm_northing, altitude);
      }
    }

    // Downsample
    double downsample_res = this->get_parameter("downsample_resolution").as_double();
    if (downsample_res > 0.0) {
      pcl::VoxelGrid<PointT> voxelgrid;
      voxelgrid.setLeafSize(downsample_res, downsample_res, downsample_res);
      voxelgrid.setInputCloud(cloud);

      auto filtered = std::make_shared<pcl::PointCloud<PointT>>();
      voxelgrid.filter(*filtered);
      cloud = filtered;

      RCLCPP_INFO(this->get_logger(),
        "Downsampled to %zu points (resolution=%.2f)", cloud->size(), downsample_res);
    }

    // Publish
    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud, *msg);
    msg->header.frame_id = "map";
    msg->header.stamp = this->now();
    globalmap_pub_->publish(std::move(msg));

    RCLCPP_INFO(this->get_logger(), "Published global map");
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_pub_;
  rclcpp::TimerBase::SharedPtr repub_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalmapServerNode>());
  rclcpp::shutdown();
  return 0;
}
