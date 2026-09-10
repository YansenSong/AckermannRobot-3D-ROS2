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

#include "ackermann_smac_bridge/ackermann_smac_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"

namespace ackermann_smac_bridge
{

AckermannSmacBridge::AckermannSmacBridge(const rclcpp::NodeOptions & options)
: Node("ackermann_smac_bridge", options)
{
  global_frame_ = declare_parameter<std::string>("global_frame", "map");
  robot_frame_ = declare_parameter<std::string>("robot_frame", "rear_axle_link");
  action_name_ = declare_parameter<std::string>(
    "planner_action", "/compute_path_to_pose");
  planner_id_ = declare_parameter<std::string>("planner_id", "GridBased");
  goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
  plan_path_topic_ = declare_parameter<std::string>("plan_path_topic", "/plan_path");
  remaining_distance_topic_ = declare_parameter<std::string>(
    "remaining_distance_topic", "/global_path_remaining_distance");
  tf_timeout_ = declare_parameter<double>("tf_timeout", 0.2);
  publish_rate_ = declare_parameter<double>("publish_rate", 10.0);
  action_server_wait_timeout_ = declare_parameter<double>(
    "action_server_wait_timeout", 2.0);

  if (global_frame_.empty() || robot_frame_.empty() || action_name_.empty()) {
    throw std::invalid_argument("global_frame, robot_frame and planner_action must not be empty");
  }
  if (tf_timeout_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "tf_timeout must be positive; using 0.2 seconds");
    tf_timeout_ = 0.2;
  }
  if (publish_rate_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "publish_rate must be positive; using 10 Hz");
    publish_rate_ = 10.0;
  }
  if (action_server_wait_timeout_ < 0.0) {
    RCLCPP_WARN(
      get_logger(), "action_server_wait_timeout must be non-negative; using 2 seconds");
    action_server_wait_timeout_ = 2.0;
  }

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, qos, std::bind(&AckermannSmacBridge::onGoal, this, std::placeholders::_1));
  plan_path_pub_ = create_publisher<nav_msgs::msg::Path>(plan_path_topic_, qos);
  remaining_distance_pub_ = create_publisher<std_msgs::msg::Float64>(
    remaining_distance_topic_, qos);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  action_client_ = rclcpp_action::create_client<ComputePathToPose>(this, action_name_);

  const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_));
  remaining_distance_timer_ = create_wall_timer(
    std::max(std::chrono::milliseconds(1), period),
    std::bind(&AckermannSmacBridge::publishRemainingDistance, this));

  active_path_.header.frame_id = global_frame_;

  RCLCPP_INFO(
    get_logger(),
    "Bridging %s goals to %s using planner '%s'; planning frame=%s, robot frame=%s",
    goal_topic_.c_str(), action_name_.c_str(), planner_id_.c_str(),
    global_frame_.c_str(), robot_frame_.c_str());
}

void AckermannSmacBridge::onGoal(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  GoalHandle::SharedPtr previous_goal;
  std::uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    generation = ++goal_generation_;
    previous_goal = active_goal_handle_;
    active_goal_handle_.reset();
    active_path_ = nav_msgs::msg::Path();
    active_path_.header.frame_id = global_frame_;

    // Clear the compatibility path atomically with the generation change so a
    // late result from the previous goal cannot repopulate it after this call.
    nav_msgs::msg::Path empty_path = active_path_;
    empty_path.header.stamp = now();
    plan_path_pub_->publish(empty_path);
  }

  if (previous_goal) {
    (void)action_client_->async_cancel_goal(previous_goal);
  }

  geometry_msgs::msg::PoseStamped goal;
  geometry_msgs::msg::PoseStamped start;
  try {
    goal = transformGoal(*msg);
    start = currentRobotPose();
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "Cannot prepare Smac goal: %s", ex.what());
    return;
  }

  if (!action_client_->wait_for_action_server(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(action_server_wait_timeout_))))
  {
    RCLCPP_ERROR(
      get_logger(), "Planner action '%s' is not available", action_name_.c_str());
    return;
  }

  ComputePathToPose::Goal action_goal;
  action_goal.start = std::move(start);
  action_goal.goal = std::move(goal);
  action_goal.planner_id = planner_id_;
  action_goal.use_start = true;

  ActionClient::SendGoalOptions options;
  options.goal_response_callback =
    [this, generation](GoalHandle::SharedPtr goal_handle) {
      onGoalResponse(generation, std::move(goal_handle));
    };
  options.result_callback =
    [this, generation](const GoalHandle::WrappedResult & wrapped_result) {
      onResult(generation, wrapped_result);
    };

  (void)action_client_->async_send_goal(action_goal, options);
}

void AckermannSmacBridge::onGoalResponse(
  const std::uint64_t generation, GoalHandle::SharedPtr goal_handle)
{
  if (!goal_handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation == goal_generation_) {
      RCLCPP_ERROR(get_logger(), "PlannerServer rejected the Smac planning goal");
    }
    return;
  }

  bool stale = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stale = generation != goal_generation_;
    if (!stale) {
      active_goal_handle_ = goal_handle;
    }
  }

  if (stale) {
    (void)action_client_->async_cancel_goal(goal_handle);
  }
}

void AckermannSmacBridge::onResult(
  const std::uint64_t generation,
  const GoalHandle::WrappedResult & wrapped_result)
{
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
    !wrapped_result.result || wrapped_result.result->path.poses.empty())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != goal_generation_) {
      return;
    }

    active_goal_handle_.reset();
    active_path_ = nav_msgs::msg::Path();
    active_path_.header.frame_id = global_frame_;
    active_path_.header.stamp = now();
    plan_path_pub_->publish(active_path_);
    RCLCPP_WARN(
      get_logger(), "Smac planning failed or returned an empty path (result code=%d)",
      static_cast<int>(wrapped_result.code));
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (generation != goal_generation_) {
    return;
  }

  active_goal_handle_.reset();
  // Preserve Smac's complete path, including orientation at Reeds-Shepp
  // reverse segments. The bridge deliberately does not recompute yaw.
  active_path_ = wrapped_result.result->path;
  plan_path_pub_->publish(active_path_);

  RCLCPP_INFO(
    get_logger(), "Smac planning succeeded: %zu poses in %.3f seconds",
    active_path_.poses.size(),
    static_cast<double>(wrapped_result.result->planning_time.sec) +
    static_cast<double>(wrapped_result.result->planning_time.nanosec) * 1e-9);
}

geometry_msgs::msg::PoseStamped AckermannSmacBridge::transformGoal(
  const geometry_msgs::msg::PoseStamped & input) const
{
  auto goal = input;
  if (goal.header.frame_id.empty() || goal.header.frame_id == global_frame_) {
    goal.header.frame_id = global_frame_;
    return goal;
  }

  const auto transform = tf_buffer_->lookupTransform(
    global_frame_, goal.header.frame_id, tf2::TimePointZero,
    tf2::durationFromSec(tf_timeout_));
  geometry_msgs::msg::PoseStamped transformed;
  tf2::doTransform(goal, transformed, transform);
  transformed.header.frame_id = global_frame_;
  return transformed;
}

geometry_msgs::msg::PoseStamped AckermannSmacBridge::currentRobotPose() const
{
  const auto transform = tf_buffer_->lookupTransform(
    global_frame_, robot_frame_, tf2::TimePointZero,
    tf2::durationFromSec(tf_timeout_));

  geometry_msgs::msg::PoseStamped pose;
  pose.header = transform.header;
  pose.header.frame_id = global_frame_;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

double AckermannSmacBridge::remainingDistance(
  const nav_msgs::msg::Path & path, const double x, const double y)
{
  if (path.poses.empty()) {
    return 0.0;
  }
  if (path.poses.size() == 1) {
    return std::hypot(
      x - path.poses.front().pose.position.x,
      y - path.poses.front().pose.position.y);
  }

  std::vector<double> cumulative(path.poses.size(), 0.0);
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    cumulative[i] = cumulative[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
  }

  double best_distance_squared = std::numeric_limits<double>::max();
  double best_remaining = cumulative.back();
  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & a = path.poses[i].pose.position;
    const auto & b = path.poses[i + 1].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;
    const double t = length_squared > 0.0 ? std::clamp(
      ((x - a.x) * dx + (y - a.y) * dy) / length_squared, 0.0, 1.0) : 0.0;
    const double projected_x = a.x + t * dx;
    const double projected_y = a.y + t * dy;
    const double distance_squared =
      (x - projected_x) * (x - projected_x) +
      (y - projected_y) * (y - projected_y);
    const double segment_length = std::sqrt(length_squared);
    const double candidate_remaining =
      cumulative.back() - (cumulative[i] + t * segment_length);
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      best_remaining = candidate_remaining;
    }
  }

  return std::max(0.0, best_remaining);
}

void AckermannSmacBridge::publishRemainingDistance()
{
  nav_msgs::msg::Path path;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_path_.poses.empty()) {
      return;
    }
    path = active_path_;
  }

  try {
    const auto pose = currentRobotPose();
    std_msgs::msg::Float64 message;
    message.data = remainingDistance(
      path, pose.pose.position.x, pose.pose.position.y);
    remaining_distance_pub_->publish(message);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Cannot compute remaining path distance: %s", ex.what());
  }
}

}  // namespace ackermann_smac_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ackermann_smac_bridge::AckermannSmacBridge>());
  rclcpp::shutdown();
  return 0;
}
