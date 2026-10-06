#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "nav_status/msg/navigation_status.hpp"

namespace nav_status
{

class NavStatusNode : public rclcpp::Node
{
public:
  using NavigationStatus = nav_status::msg::NavigationStatus;

  enum class State : std::uint8_t
  {
    WAITING_FOR_GOAL = NavigationStatus::WAITING_FOR_GOAL,
    PLANNING = NavigationStatus::PLANNING,
    MOVING = NavigationStatus::MOVING,
    ARRIVED = NavigationStatus::ARRIVED,
    FAILED = NavigationStatus::FAILED,
  };

  explicit NavStatusNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("nav_status_node", options), state_(State::WAITING_FOR_GOAL)
  {
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
    plan_topic_ = declare_parameter<std::string>("plan_topic", "/plan_path");
    remaining_distance_topic_ = declare_parameter<std::string>(
      "remaining_distance_topic", "/global_path_remaining_distance");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom_wheel");
    status_topic_ = declare_parameter<std::string>("status_topic", "/navigation/state");

    arrival_distance_threshold_ = declare_parameter<double>(
      "arrival_distance_threshold", 0.20);
    stopped_speed_threshold_ = declare_parameter<double>(
      "stopped_speed_threshold", 0.05);
    arrival_hold_time_ = declare_parameter<double>("arrival_hold_time", 0.80);
    input_timeout_ = declare_parameter<double>("input_timeout", 1.0);
    evaluation_rate_ = declare_parameter<double>("evaluation_rate", 20.0);
    status_heartbeat_rate_ = declare_parameter<double>("status_heartbeat_rate", 1.0);

    if (arrival_distance_threshold_ < 0.0) {
      RCLCPP_WARN(this->get_logger(),
        "arrival_distance_threshold must be non-negative; using 0.20 m");
      arrival_distance_threshold_ = 0.20;
    }
    if (stopped_speed_threshold_ < 0.0) {
      RCLCPP_WARN(this->get_logger(),
        "stopped_speed_threshold must be non-negative; using 0.05 m/s");
      stopped_speed_threshold_ = 0.05;
    }
    if (arrival_hold_time_ < 0.0) {
      RCLCPP_WARN(this->get_logger(),
        "arrival_hold_time must be non-negative; using 0.80 s");
      arrival_hold_time_ = 0.80;
    }
    if (input_timeout_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
        "input_timeout must be positive; using 1.0 s");
      input_timeout_ = 1.0;
    }
    if (evaluation_rate_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
        "evaluation_rate must be positive; using 20.0 Hz");
      evaluation_rate_ = 20.0;
    }
    if (status_heartbeat_rate_ < 0.0) {
      RCLCPP_WARN(this->get_logger(),
        "status_heartbeat_rate must be non-negative; disabling heartbeat");
      status_heartbeat_rate_ = 0.0;
    }

    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    status_qos.reliable();
    status_qos.transient_local();
    status_pub_ = create_publisher<NavigationStatus>(status_topic_, status_qos);

    const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, input_qos,
      std::bind(&NavStatusNode::onGoal, this, std::placeholders::_1));
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic_, input_qos,
      std::bind(&NavStatusNode::onPlan, this, std::placeholders::_1));
    remaining_distance_sub_ = create_subscription<std_msgs::msg::Float64>(
      remaining_distance_topic_, input_qos,
      std::bind(&NavStatusNode::onRemainingDistance, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, input_qos,
      std::bind(&NavStatusNode::onOdom, this, std::placeholders::_1));

    evaluation_timer_ = create_wall_timer(
      periodFromRate(evaluation_rate_),
      std::bind(&NavStatusNode::evaluateArrival, this));
    if (status_heartbeat_rate_ > 0.0) {
      heartbeat_timer_ = create_wall_timer(
        periodFromRate(status_heartbeat_rate_),
        std::bind(&NavStatusNode::publishStatus, this));
    }

    detail_ = "node started; waiting for goal";
    publishStatus();
  }

private:
  static std::chrono::milliseconds periodFromRate(const double rate)
  {
    const auto period_ms = static_cast<std::int64_t>(std::ceil(1000.0 / rate));
    return std::chrono::milliseconds(std::max<std::int64_t>(1, period_ms));
  }

  static const char * stateName(const State state)
  {
    switch (state) {
      case State::WAITING_FOR_GOAL:
        return "WAITING_FOR_GOAL";
      case State::PLANNING:
        return "PLANNING";
      case State::MOVING:
        return "MOVING";
      case State::ARRIVED:
        return "ARRIVED";
      case State::FAILED:
        return "FAILED";
    }
    return "UNKNOWN";
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr /*msg*/)
  {
    arrival_candidate_since_.reset();

    // Do not reuse telemetry from the previous navigation task.  Odom and
    // remaining-distance callbacks will repopulate these values immediately.
    remaining_distance_.reset();
    remaining_distance_stamp_.reset();
    actual_speed_.reset();
    odom_stamp_.reset();

    transitionTo(State::PLANNING, "new goal received; waiting for global plan");
  }

  void onPlan(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (state_ != State::PLANNING || msg->poses.empty()) {
      return;
    }

    arrival_candidate_since_.reset();
    transitionTo(State::MOVING, "global plan available; navigation active");
  }

  void onRemainingDistance(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      return;
    }

    remaining_distance_ = msg->data;
    remaining_distance_stamp_ = now();
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double speed = std::hypot(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y);
    if (!std::isfinite(speed)) {
      return;
    }

    actual_speed_ = speed;
    odom_stamp_ = now();
  }

  bool isFresh(const std::optional<rclcpp::Time> & stamp, const rclcpp::Time & current) const
  {
    if (!stamp) {
      return false;
    }

    const double age = (current - *stamp).seconds();
    return age >= 0.0 && age <= input_timeout_;
  }

  void evaluateArrival()
  {
    if (state_ != State::MOVING) {
      arrival_candidate_since_.reset();
      return;
    }

    const auto current = now();
    if (!remaining_distance_ || !actual_speed_ ||
      !isFresh(remaining_distance_stamp_, current) || !isFresh(odom_stamp_, current))
    {
      arrival_candidate_since_.reset();
      return;
    }

    const bool close_enough = *remaining_distance_ <= arrival_distance_threshold_;
    const bool stopped = *actual_speed_ <= stopped_speed_threshold_;
    if (!close_enough || !stopped) {
      arrival_candidate_since_.reset();
      return;
    }

    if (!arrival_candidate_since_) {
      arrival_candidate_since_ = current;
      return;
    }

    if ((current - *arrival_candidate_since_).seconds() >= arrival_hold_time_) {
      transitionTo(State::ARRIVED, "goal reached and vehicle stopped");
      arrival_candidate_since_.reset();
    }
  }

  void transitionTo(const State next, const std::string & detail)
  {
    const State previous = state_;
    const bool changed = previous != next;
    state_ = next;
    detail_ = detail;

    if (changed) {
      RCLCPP_INFO(this->get_logger(),
        "Navigation state: %s -> %s",
        stateName(previous), stateName(next));
    }

    // A new goal can arrive while already in PLANNING.  Republish in that
    // case so consumers can observe the new task even without a state change.
    publishStatus();
  }

  void publishStatus()
  {
    NavigationStatus status;
    status.stamp = now();
    status.state = static_cast<std::uint8_t>(state_);
    status.detail = detail_;
    status_pub_->publish(status);
  }

  State state_;
  std::string detail_;

  std::optional<double> remaining_distance_;
  std::optional<double> actual_speed_;
  std::optional<rclcpp::Time> remaining_distance_stamp_;
  std::optional<rclcpp::Time> odom_stamp_;
  std::optional<rclcpp::Time> arrival_candidate_since_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr remaining_distance_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<NavigationStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr evaluation_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  std::string goal_topic_;
  std::string plan_topic_;
  std::string remaining_distance_topic_;
  std::string odom_topic_;
  std::string status_topic_;

  double arrival_distance_threshold_ = 0.20;
  double stopped_speed_threshold_ = 0.05;
  double arrival_hold_time_ = 0.80;
  double input_timeout_ = 1.0;
  double evaluation_rate_ = 20.0;
  double status_heartbeat_rate_ = 1.0;
};

}  // namespace nav_status

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav_status::NavStatusNode>());
  rclcpp::shutdown();
  return 0;
}
