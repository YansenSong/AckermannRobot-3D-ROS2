/**
 * Hybrid A* 全局路径规划器 — ROS2 节点 (适配 AckermannRobot-3D)
 *
 * - 加载 PGM 栅格地图，无需 Nav2 costmap
 * - 通过 TF 查 map→rear_axle_link 获取当前位姿作为规划起点
 * - 暴露 /plan 话题 (NeuPAN 订阅) 和 /plan 服务 (GetPlan)
 * - 发布 /map (OccupancyGrid, latched) 供 RViz2 显示
 *
 * TF 树: map ←(hdl NDT)← odom ←(EKF)← base_link ←(URDF)← rear_axle_link
 * hdl_localization 负责 map→odom，本节点不广播 TF。
 *
 * 车辆: 小型阿克曼, 0.70m×0.52m, wheelbase=0.593m, min turning radius=1.05m
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "hybrid_astar_planner/pgm_map_adapter.hpp"
#include "node_se2.hpp"
#include "a_star.hpp"
#include "collision_checker.hpp"
#include "smoother.hpp"
#include "types.hpp"
#include "constants.hpp"
#include "options.hpp"

using namespace std::placeholders;
using namespace nav2_smac_planner;

// ---- 用于 footprint 顶点的 Point 类型 (需要 x,y 成员) ----
struct Point2D { double x; double y; };

class HybridAStarPlannerNode : public rclcpp::Node
{
public:
  using CostmapT          = hybrid_astar_planner::PGMMapAdapter;
  using CollisionCheckerT = nav2_smac_planner::GridCollisionChecker<CostmapT, Point2D>;
  using AStarT            = nav2_smac_planner::AStarAlgorithm<CostmapT, CollisionCheckerT>;
  using SmootherT         = nav2_smac_planner::Smoother<CostmapT>;
  using Footprint         = typename CollisionCheckerT::Footprint;
  using CoordinateVector  = typename nav2_smac_planner::NodeSE2::CoordinateVector;

  explicit HybridAStarPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("hybrid_astar_planner", options)
  {
    loadParams();
    loadMap();

    // 构建 A* 引擎
    a_star_   = std::make_unique<AStarT>(_motion_model, _search_info);
    smoother_ = std::make_unique<SmootherT>();

    // TF 监听器 — 查 map→robot_state_frame 获取当前位姿
    // (hdl_localization 仍通过 base_link 提供 map→odom)
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // GetPlan 服务 (/plan)
    plan_srv_ = this->create_service<nav_msgs::srv::GetPlan>(
      "/plan",
      std::bind(&HybridAStarPlannerNode::planCallback, this, _1, _2));

    // 路径发布话题 — /plan (NeuPAN 订阅) + /plan_path (RViz)
    plan_pub_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/plan_path", 10);
    remaining_distance_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      _remaining_distance_topic, 10);
    remaining_distance_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&HybridAStarPlannerNode::publishRemainingDistance, this));

    // 地图发布 (latched + 定时重发, 确保 RViz2 后启动也能收到)
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    publishMap();
    map_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&HybridAStarPlannerNode::publishMap, this));

    // 订阅 RViz 2D Goal Pose → 自动规划
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&HybridAStarPlannerNode::goalPoseCallback, this, _1));

    // 初始化 A* 图结构
    initGraph();

    RCLCPP_INFO(this->get_logger(),
      "Hybrid A* Planner ready. Map: %ux%u cells, turning radius: %.1f cells (%.2f m)",
      map_->getSizeInCellsX(), map_->getSizeInCellsY(),
      _min_turning_radius_cells, _min_turning_radius_m);
  }

private:
  // ================================================================
  // 参数加载
  // ================================================================
  void loadParams()
  {
    // ---- 地图 ----
    map_path_  = this->declare_parameter("map_path", "");
    resolution_ = this->declare_parameter("resolution", 0.05);
    origin_x_  = this->declare_parameter("origin_x",  0.0);
    origin_y_  = this->declare_parameter("origin_y",  0.0);

    // ---- 路径规划 (小型阿克曼: turning radius = 1.05m) ----
    _min_turning_radius_m = this->declare_parameter("minimum_turning_radius", 1.05);
    _angle_quantization   = this->declare_parameter("angle_quantization",     72);
    _max_iterations       = this->declare_parameter("max_iterations",         50000);
    _max_on_approach_it   = this->declare_parameter("max_on_approach_iterations", 500);
    _tolerance_meters     = this->declare_parameter("tolerance",              0.5);
    _allow_unknown        = this->declare_parameter("allow_unknown",          true);

    // ---- 搜索惩罚 ----
    _search_info.change_penalty       = this->declare_parameter("change_penalty",        1.0f);
    _search_info.non_straight_penalty = this->declare_parameter("non_straight_penalty",  1.2f);
    _search_info.reverse_penalty      = this->declare_parameter("reverse_penalty",       2.1f);
    _search_info.cost_penalty         = this->declare_parameter("cost_penalty",          1.0f);
    _search_info.analytic_expansion_ratio =
      this->declare_parameter("analytic_expansion_ratio", 3.0f);

    // ---- 运动模型 ----
    std::string model_str = this->declare_parameter("motion_model", std::string("REEDS_SHEPP"));
    _motion_model = (model_str == "DUBIN") ? MotionModel::DUBIN : MotionModel::REEDS_SHEPP;

    // ---- 平滑器 (默认关闭) ----
    _use_smoother            = this->declare_parameter("use_smoother", false);
    _smoother_params.max_time = this->declare_parameter("smooth_max_time", 0.1);
    _optimizer_params.max_iterations = this->declare_parameter("smooth_max_iterations", 50);

    // ---- 车体尺寸 (0.70m×0.52m, 后轴在几何中心后方) ----
    _vehicle_length = this->declare_parameter("vehicle_length", 0.70);
    _vehicle_width  = this->declare_parameter("vehicle_width",  0.52);
    _robot_state_frame = this->declare_parameter(
      "robot_state_frame", std::string("base_link"));
    _goal_pose_is_base_link = this->declare_parameter("goal_pose_is_base_link", true);
    _rear_axle_offset_x = this->declare_parameter("rear_axle_offset_x", -0.25);
    _remaining_distance_topic = this->declare_parameter(
      "remaining_distance_topic", std::string("/global_path_remaining_distance"));

    // ---- 分析扩展 ----
    _search_info.analytic_expansion_max_length =
      static_cast<float>(this->declare_parameter("analytic_expansion_max_length", 50.0));
  }

  // ================================================================
  // 地图加载
  // ================================================================
  void loadMap()
  {
    map_ = std::make_unique<CostmapT>();
    _min_turning_radius_cells = static_cast<float>(_min_turning_radius_m / resolution_);
    _search_info.minimum_turning_radius = _min_turning_radius_cells;

    if (!map_->load(map_path_, resolution_, origin_x_, origin_y_)) {
      throw std::runtime_error("Failed to load map: " + map_path_);
    }
  }

  // ================================================================
  // A* 图初始化 (每次规划前调用)
  // ================================================================
  void initGraph()
  {
    Footprint footprint;
    double rear_to_axle  = _vehicle_length / 2.0 + _rear_axle_offset_x;
    double front_to_axle = _vehicle_length / 2.0 - _rear_axle_offset_x;
    double hw = _vehicle_width / 2.0;
    footprint.push_back({-rear_to_axle, -hw});
    footprint.push_back({ front_to_axle, -hw});
    footprint.push_back({ front_to_axle,  hw});
    footprint.push_back({-rear_to_axle,  hw});

    a_star_->initialize(_allow_unknown, _max_iterations, _max_on_approach_it);
    a_star_->setFootprint(footprint, false);

    auto * costmap_ptr = map_.get();
    a_star_->createGraph(
      map_->getSizeInCellsX(),
      map_->getSizeInCellsY(),
      static_cast<unsigned int>(_angle_quantization),
      costmap_ptr);
  }

  // ================================================================
  // TF 查 map→robot_state_frame, 返回 map 坐标系下的当前位姿
  // ================================================================
  bool lookupMapToRobotState(geometry_msgs::msg::PoseStamped & pose_out)
  {
    try {
      auto transform = tf_buffer_->lookupTransform(
        "map", _robot_state_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
      pose_out.header = transform.header;
      pose_out.header.frame_id = "map";
      pose_out.pose.position.x = transform.transform.translation.x;
      pose_out.pose.position.y = transform.transform.translation.y;
      pose_out.pose.position.z = 0.0;
      pose_out.pose.orientation = transform.transform.rotation;
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "TF lookup map→%s failed: %s", _robot_state_frame.c_str(), e.what());
      return false;
    }
  }

  // RViz's 2D Goal Pose represents the vehicle/base pose.  When the planner
  // state is the rear axle, shift that target by the configured fixed offset
  // along the target heading so that the vehicle center reaches the clicked
  // goal after the rear axle reaches the converted target.
  geometry_msgs::msg::PoseStamped goalToPlanningReference(
    const geometry_msgs::msg::PoseStamped & goal) const
  {
    auto converted = goal;
    if (!_goal_pose_is_base_link || _robot_state_frame == "base_link") {
      return converted;
    }

    tf2::Quaternion tf_q(
      goal.pose.orientation.x, goal.pose.orientation.y,
      goal.pose.orientation.z, goal.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    converted.pose.position.x += _rear_axle_offset_x * std::cos(yaw);
    converted.pose.position.y += _rear_axle_offset_x * std::sin(yaw);
    return converted;
  }

  // ================================================================
  // GetPlan 服务回调
  // ================================================================
  void planCallback(
    const std::shared_ptr<nav_msgs::srv::GetPlan::Request>  req,
    std::shared_ptr<nav_msgs::srv::GetPlan::Response>        res)
  {
    RCLCPP_INFO(this->get_logger(),
      "Plan request: start=(%.2f, %.2f) goal=(%.2f, %.2f)",
      req->start.pose.position.x, req->start.pose.position.y,
      req->goal.pose.position.x,  req->goal.pose.position.y);

    try {
      res->plan = doPlan(req->start, req->goal, req->start.header);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Planning exception: %s", e.what());
      res->plan = makeEmptyPath(req->start.header);
    }
  }

  // ================================================================
  // RViz 2D Goal Pose 回调 — 自动规划并发布路径
  // ================================================================
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
  {
    geometry_msgs::msg::PoseStamped start;
    if (!lookupMapToRobotState(start)) {
      RCLCPP_WARN(this->get_logger(),
        "No TF map→%s yet, cannot plan. Has hdl_localization started?",
        _robot_state_frame.c_str());
      return;
    }

    const auto planning_goal = goalToPlanningReference(*goal);

    RCLCPP_INFO(this->get_logger(),
      "Goal: (%.2f, %.2f), start: (%.2f, %.2f) [state frame: %s]",
      planning_goal.pose.position.x, planning_goal.pose.position.y,
      start.pose.position.x, start.pose.position.y,
      _robot_state_frame.c_str());

    auto path = doPlan(start, planning_goal, start.header);
    if (!path.poses.empty()) {
      {
        std::lock_guard<std::mutex> lock(active_path_mutex_);
        active_path_ = path;
      }
      // 发布到 /plan (NeuPAN) 和 /plan_path (RViz)
      plan_pub_->publish(path);
      path_pub_->publish(path);
    }
  }

  /** 核心规划: start + goal PoseStamped → Path */
  nav_msgs::msg::Path doPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std_msgs::msg::Header & header_template)
  {
    initGraph();

    unsigned int sx, sy, gx, gy;
    if (!map_->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy)) {
      RCLCPP_ERROR(this->get_logger(), "Start pose out of map bounds");
      return makeEmptyPath(header_template);
    }
    if (!map_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
      RCLCPP_ERROR(this->get_logger(), "Goal pose out of map bounds");
      return makeEmptyPath(header_template);
    }

    unsigned int s_bin = yawToBin(start.pose.orientation);
    unsigned int g_bin = yawToBin(goal.pose.orientation);

    a_star_->setStart(sx, sy, s_bin);
    a_star_->setGoal(gx, gy, g_bin);

    int    num_iterations  = 0;
    float  tolerance_cells = static_cast<float>(_tolerance_meters / resolution_);
    CoordinateVector raw_path;

    bool found = a_star_->createPath(raw_path, num_iterations, tolerance_cells);

    RCLCPP_INFO(this->get_logger(),
      "Plan %s: %d iterations, path size=%zu",
      found ? "OK" : "FAILED", num_iterations, raw_path.size());

    if (found) {
      if (_use_smoother && raw_path.size() > 10) {
        raw_path = smoothPath(raw_path);
      }
      return gridPathToROSPath(raw_path, header_template);
    }
    return makeEmptyPath(header_template);
  }

  // ================================================================
  // 工具函数
  // ================================================================

  unsigned int yawToBin(const geometry_msgs::msg::Quaternion & q) const
  {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    while (yaw < 0)        yaw += 2.0 * M_PI;
    while (yaw >= 2.0*M_PI) yaw -= 2.0 * M_PI;
    double bin_size = 2.0 * M_PI / _angle_quantization;
    return static_cast<unsigned int>(std::round(yaw / bin_size)) % _angle_quantization;
  }

  nav_msgs::msg::Path gridPathToROSPath(
    const CoordinateVector & grid_path,
    const std_msgs::msg::Header & header_template) const
  {
    nav_msgs::msg::Path path;
    path.header = header_template;
    path.header.frame_id = "map";

    // backtrace 是 goal → start, 需要反转
    for (auto it = grid_path.rbegin(); it != grid_path.rend(); ++it) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      double wx, wy;
      map_->mapToWorldContinuous(it->x, it->y, wx, wy);
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;

      double theta = it->theta * (2.0 * M_PI / _angle_quantization);
      tf2::Quaternion q;
      q.setRPY(0, 0, theta);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();

      path.poses.push_back(pose);
    }

    return path;
  }

  nav_msgs::msg::Path makeEmptyPath(const std_msgs::msg::Header & h) const
  {
    nav_msgs::msg::Path path;
    path.header = h;
    path.header.frame_id = "map";
    return path;
  }

  // 计算当前位置投影到全局路径后，沿路径到终点的剩余距离。
  // 这不是当前位置到目标点的直线距离，而是路径曲线上的距离。
  double calculateRemainingDistance(
    const nav_msgs::msg::Path & path, double current_x, double current_y) const
  {
    if (path.poses.empty()) {
      return 0.0;
    }

    if (path.poses.size() == 1) {
      const auto & goal = path.poses.front().pose.position;
      return std::hypot(goal.x - current_x, goal.y - current_y);
    }

    const size_t segment_count = path.poses.size() - 1;
    std::vector<double> segment_lengths(segment_count, 0.0);
    std::vector<double> suffix_lengths(path.poses.size(), 0.0);

    for (size_t i = 0; i < segment_count; ++i) {
      const auto & p0 = path.poses[i].pose.position;
      const auto & p1 = path.poses[i + 1].pose.position;
      segment_lengths[i] = std::hypot(p1.x - p0.x, p1.y - p0.y);
    }
    for (size_t i = segment_count; i > 0; --i) {
      suffix_lengths[i - 1] = segment_lengths[i - 1] + suffix_lengths[i];
    }

    constexpr double epsilon = 1e-9;
    double best_distance_squared = std::numeric_limits<double>::max();
    double best_remaining_distance = suffix_lengths.front();

    for (size_t i = 0; i < segment_count; ++i) {
      const auto & p0 = path.poses[i].pose.position;
      const auto & p1 = path.poses[i + 1].pose.position;
      const double dx = p1.x - p0.x;
      const double dy = p1.y - p0.y;
      const double length = segment_lengths[i];

      double t = 0.0;
      if (length > epsilon) {
        t = ((current_x - p0.x) * dx + (current_y - p0.y) * dy) /
          (length * length);
        t = std::max(0.0, std::min(1.0, t));
      }

      const double projected_x = p0.x + t * dx;
      const double projected_y = p0.y + t * dy;
      const double distance_x = current_x - projected_x;
      const double distance_y = current_y - projected_y;
      const double distance_squared = distance_x * distance_x + distance_y * distance_y;

      if (distance_squared < best_distance_squared) {
        best_distance_squared = distance_squared;
        best_remaining_distance = (1.0 - t) * length + suffix_lengths[i + 1];
      }
    }

    return std::max(0.0, best_remaining_distance);
  }

  void publishRemainingDistance()
  {
    nav_msgs::msg::Path path;
    {
      std::lock_guard<std::mutex> lock(active_path_mutex_);
      path = active_path_;
    }

    if (path.poses.empty()) {
      return;
    }

    geometry_msgs::msg::PoseStamped current;
    if (!lookupMapToRobotState(current)) {
      return;
    }

    std_msgs::msg::Float64 remaining;
    remaining.data = calculateRemainingDistance(
      path, current.pose.position.x, current.pose.position.y);
    remaining_distance_pub_->publish(remaining);
  }

  void publishMap()
  {
    auto grid = nav_msgs::msg::OccupancyGrid();
    grid.header.frame_id = "map";
    grid.header.stamp    = this->now();
    grid.info.resolution = map_->getResolution();
    grid.info.width      = map_->getSizeInCellsX();
    grid.info.height     = map_->getSizeInCellsY();
    grid.info.origin.position.x = map_->getOriginX();
    grid.info.origin.position.y = map_->getOriginY();
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    const auto & raw = map_->getRawData();
    grid.data.resize(raw.size());
    for (size_t i = 0; i < raw.size(); ++i) {
      switch (raw[i]) {
        case 0:   grid.data[i] = 100; break;
        case 254: grid.data[i] = 0;   break;
        case 205: grid.data[i] = -1;  break;
        default:  grid.data[i] = -1;  break;
      }
    }
    map_pub_->publish(grid);
  }

  CoordinateVector smoothPath(const CoordinateVector & raw)
  {
    std::vector<Eigen::Vector2d> eigen_path;
    eigen_path.reserve(raw.size());
    for (const auto & c : raw) {
      double wx, wy;
      map_->mapToWorldContinuous(c.x, c.y, wx, wy);
      eigen_path.emplace_back(wx, wy);
    }

    SmootherParams s_params;
    s_params.max_time = _smoother_params.max_time;
    OptimizerParams o_params;
    o_params.max_iterations = _optimizer_params.max_iterations;

    smoother_->initialize(o_params);
    smoother_->smooth(eigen_path, map_.get(), s_params);

    CoordinateVector smoothed;
    smoothed.reserve(eigen_path.size());
    unsigned int mx, my;
    for (const auto & pt : eigen_path) {
      if (map_->worldToMap(pt[0], pt[1], mx, my)) {
        float theta = 0.0f;
        if (!smoothed.empty()) {
          auto & prev = smoothed.back();
          double dx = static_cast<double>(mx) - prev.x;
          double dy = static_cast<double>(my) - prev.y;
          theta = static_cast<float>(std::atan2(dy, dx) / (2.0 * M_PI / _angle_quantization));
          while (theta < 0) theta += static_cast<float>(_angle_quantization);
        }
        smoothed.push_back({static_cast<float>(mx), static_cast<float>(my), theta});
      }
    }
    return smoothed;
  }

  // ================================================================
  // 成员变量
  // ================================================================
  std::unique_ptr<CostmapT>    map_;
  std::unique_ptr<AStarT>      a_star_;
  std::unique_ptr<SmootherT>   smoother_;

  // TF (hdl_localization 负责 map→odom，本节点只监听)
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr        plan_srv_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr          plan_pub_;   // /plan (NeuPAN)
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr          path_pub_;   // /plan_path (RViz)
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       remaining_distance_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr remaining_distance_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  nav_msgs::msg::Path active_path_;
  std::mutex active_path_mutex_;

  // 配置参数
  std::string  map_path_;
  std::string  _robot_state_frame = "base_link";
  double       resolution_   = 0.05;
  double       origin_x_     = 0.0;
  double       origin_y_     = 0.0;
  double       _min_turning_radius_m = 1.05;
  int          _angle_quantization   = 72;
  int          _max_iterations       = 50000;
  int          _max_on_approach_it   = 500;
  double       _tolerance_meters     = 0.5;
  bool         _allow_unknown        = true;
  bool         _use_smoother         = false;
  bool         _goal_pose_is_base_link = true;
  double       _vehicle_length       = 0.70;
  double       _vehicle_width        = 0.52;
  double       _rear_axle_offset_x   = -0.25;
  std::string  _remaining_distance_topic = "/global_path_remaining_distance";
  float        _min_turning_radius_cells = 21.0f;

  MotionModel    _motion_model = MotionModel::REEDS_SHEPP;
  SearchInfo     _search_info;
  SmootherParams _smoother_params;
  OptimizerParams _optimizer_params;
};

// ---- main ----
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<HybridAStarPlannerNode>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("hybrid_astar_planner"),
      "Fatal: %s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
