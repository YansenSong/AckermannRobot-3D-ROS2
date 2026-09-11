// Copyright (c) 2026 AckermannRobot contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "nav_status/msg/navigation_status.hpp"

namespace nav_status
{

class Nav2StatusNode : public rclcpp::Node
{
public:
  using NavigationStatus = nav_status::msg::NavigationStatus;
  using GoalStatus = action_msgs::msg::GoalStatus;
  using GoalStatusArray = action_msgs::msg::GoalStatusArray;
  using GoalId = std::array<std::uint8_t, 16>;

  enum class State : std::uint8_t
  {
    WAITING_FOR_GOAL = NavigationStatus::WAITING_FOR_GOAL,
    PLANNING = NavigationStatus::PLANNING,
    MOVING = NavigationStatus::MOVING,
    ARRIVED = NavigationStatus::ARRIVED,
    FAILED = NavigationStatus::FAILED,
  };

  explicit Nav2StatusNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("nav2_status_node", options), state_(State::WAITING_FOR_GOAL)
  {
    navigate_to_pose_status_topic_ = declare_parameter<std::string>(
      "navigate_to_pose_status_topic", "/navigate_to_pose/_action/status");
    navigate_to_pose_feedback_topic_ = declare_parameter<std::string>(
      "navigate_to_pose_feedback_topic", "/navigate_to_pose/_action/feedback");
    navigate_through_poses_status_topic_ = declare_parameter<std::string>(
      "navigate_through_poses_status_topic", "/navigate_through_poses/_action/status");
    navigate_through_poses_feedback_topic_ = declare_parameter<std::string>(
      "navigate_through_poses_feedback_topic", "/navigate_through_poses/_action/feedback");
    remaining_distance_topic_ = declare_parameter<std::string>(
      "remaining_distance_topic", "/global_path_remaining_distance");
    status_topic_ = declare_parameter<std::string>("status_topic", "/navigation/state");
    status_heartbeat_rate_ = declare_parameter<double>("status_heartbeat_rate", 1.0);

    if (status_heartbeat_rate_ < 0.0) {
      RCLCPP_WARN(
        get_logger(), "status_heartbeat_rate must be non-negative; disabling heartbeat");
      status_heartbeat_rate_ = 0.0;
    }

    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    status_qos.reliable();
    status_qos.transient_local();
    status_pub_ = create_publisher<NavigationStatus>(status_topic_, status_qos);
    remaining_distance_pub_ = create_publisher<std_msgs::msg::Float64>(
      remaining_distance_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    auto action_status_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    action_status_qos.reliable();
    action_status_qos.transient_local();
    const auto action_feedback_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    status_to_pose_sub_ = create_subscription<GoalStatusArray>(
      navigate_to_pose_status_topic_, action_status_qos,
      std::bind(&Nav2StatusNode::onActionStatus, this, std::placeholders::_1));
    status_through_poses_sub_ = create_subscription<GoalStatusArray>(
      navigate_through_poses_status_topic_, action_status_qos,
      std::bind(&Nav2StatusNode::onActionStatus, this, std::placeholders::_1));
    feedback_to_pose_sub_ = create_subscription<
      nav2_msgs::action::NavigateToPose_FeedbackMessage>(
      navigate_to_pose_feedback_topic_, action_feedback_qos,
      std::bind(&Nav2StatusNode::onNavigateToPoseFeedback, this, std::placeholders::_1));
    feedback_through_poses_sub_ = create_subscription<
      nav2_msgs::action::NavigateThroughPoses_FeedbackMessage>(
      navigate_through_poses_feedback_topic_, action_feedback_qos,
      std::bind(
        &Nav2StatusNode::onNavigateThroughPosesFeedback, this, std::placeholders::_1));

    if (status_heartbeat_rate_ > 0.0) {
      heartbeat_timer_ = create_wall_timer(
        periodFromRate(status_heartbeat_rate_),
        std::bind(&Nav2StatusNode::publishStatus, this));
    }

    detail_ = "node started; waiting for Nav2 goal";
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

  static bool isActiveStatus(const std::int8_t status)
  {
    return status == GoalStatus::STATUS_ACCEPTED ||
           status == GoalStatus::STATUS_EXECUTING ||
           status == GoalStatus::STATUS_CANCELING;
  }

  static bool isTerminalStatus(const std::int8_t status)
  {
    return status == GoalStatus::STATUS_SUCCEEDED ||
           status == GoalStatus::STATUS_CANCELED ||
           status == GoalStatus::STATUS_ABORTED;
  }

  static std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
           static_cast<std::int64_t>(stamp.nanosec);
  }

  static bool sameGoal(const GoalId & first, const GoalId & second)
  {
    return first == second;
  }

  void onActionStatus(const GoalStatusArray::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    std::optional<GoalStatus> newest_active;
    for (const auto & candidate : msg->status_list) {
      if (!isActiveStatus(candidate.status)) {
        continue;
      }
      if (!newest_active ||
        stampNanoseconds(candidate.goal_info.stamp) >
        stampNanoseconds(newest_active->goal_info.stamp))
      {
        newest_active = candidate;
      }
    }

    if (newest_active) {
      const auto & goal_id = newest_active->goal_info.goal_id.uuid;
      if (!active_goal_ || !sameGoal(*active_goal_, goal_id)) {
        activateGoal(goal_id);
      }

      if (newest_active->status == GoalStatus::STATUS_CANCELING) {
        detail_ = "Nav2 goal is being canceled";
        publishStatus();
      }
      return;
    }

    if (!active_goal_) {
      return;
    }

    for (const auto & candidate : msg->status_list) {
      if (!isTerminalStatus(candidate.status) ||
        !sameGoal(*active_goal_, candidate.goal_info.goal_id.uuid))
      {
        continue;
      }

      const auto finished_goal = *active_goal_;
      active_goal_.reset();
      finished_goal_ = finished_goal;

      switch (candidate.status) {
        case GoalStatus::STATUS_SUCCEEDED:
          publishRemainingDistance(0.0);
          transitionTo(State::ARRIVED, "Nav2 goal reached successfully");
          break;
        case GoalStatus::STATUS_CANCELED:
          transitionTo(State::WAITING_FOR_GOAL, "Nav2 goal canceled");
          break;
        case GoalStatus::STATUS_ABORTED:
          transitionTo(State::FAILED, "Nav2 goal aborted");
          break;
        default:
          break;
      }
      return;
    }
  }

  void onNavigateToPoseFeedback(
    const nav2_msgs::action::NavigateToPose_FeedbackMessage::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    onFeedback(msg->goal_id.uuid, msg->feedback.distance_remaining);
  }

  void onNavigateThroughPosesFeedback(
    const nav2_msgs::action::NavigateThroughPoses_FeedbackMessage::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    onFeedback(msg->goal_id.uuid, msg->feedback.distance_remaining);
  }

  void onFeedback(const GoalId & goal_id, const float distance_remaining)
  {
    if (finished_goal_ && sameGoal(*finished_goal_, goal_id)) {
      return;
    }

    // Feedback can arrive before the status message. Treat it as evidence
    // of an accepted Nav2 goal, then use the UUID to reject stale feedback.
    if (!active_goal_) {
      activateGoal(goal_id);
    } else if (!sameGoal(*active_goal_, goal_id)) {
      return;
    }

    if (!std::isfinite(distance_remaining) || distance_remaining < 0.0F) {
      return;
    }

    publishRemainingDistance(static_cast<double>(distance_remaining));
    if (state_ != State::MOVING) {
      transitionTo(State::MOVING, "Nav2 feedback active; navigation in progress");
    }
  }

  void activateGoal(const GoalId & goal_id)
  {
    active_goal_ = goal_id;
    finished_goal_.reset();
    transitionTo(State::PLANNING, "Nav2 goal accepted; waiting for navigation feedback");
  }

  void publishRemainingDistance(const double distance)
  {
    std_msgs::msg::Float64 message;
    message.data = distance;
    remaining_distance_pub_->publish(message);
  }

  void transitionTo(const State next, const std::string & detail)
  {
    const State previous = state_;
    state_ = next;
    detail_ = detail;

    if (previous != next) {
      RCLCPP_INFO(
        get_logger(), "Navigation state: %s -> %s",
        stateName(previous), stateName(next));
    }
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
  std::optional<GoalId> active_goal_;
  std::optional<GoalId> finished_goal_;

  rclcpp::Subscription<GoalStatusArray>::SharedPtr status_to_pose_sub_;
  rclcpp::Subscription<GoalStatusArray>::SharedPtr status_through_poses_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>::SharedPtr
    feedback_to_pose_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses_FeedbackMessage>::SharedPtr
    feedback_through_poses_sub_;
  rclcpp::Publisher<NavigationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr remaining_distance_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  std::string navigate_to_pose_status_topic_;
  std::string navigate_to_pose_feedback_topic_;
  std::string navigate_through_poses_status_topic_;
  std::string navigate_through_poses_feedback_topic_;
  std::string remaining_distance_topic_;
  std::string status_topic_;
  double status_heartbeat_rate_ = 1.0;
};

}  // namespace nav_status

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav_status::Nav2StatusNode>());
  rclcpp::shutdown();
  return 0;
}
