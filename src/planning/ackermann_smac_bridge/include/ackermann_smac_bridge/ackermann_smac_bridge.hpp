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

#ifndef ACKERMANN_SMAC_BRIDGE__ACKERMANN_SMAC_BRIDGE_HPP_
#define ACKERMANN_SMAC_BRIDGE__ACKERMANN_SMAC_BRIDGE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace ackermann_smac_bridge
{

class AckermannSmacBridge final : public rclcpp::Node
{
public:
  explicit AckermannSmacBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using ActionClient = rclcpp_action::Client<ComputePathToPose>;

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onGoalResponse(std::uint64_t generation, GoalHandle::SharedPtr goal_handle);
  void onResult(
    std::uint64_t generation,
    const GoalHandle::WrappedResult & wrapped_result);
  void publishRemainingDistance();

  geometry_msgs::msg::PoseStamped transformGoal(
    const geometry_msgs::msg::PoseStamped & goal) const;
  geometry_msgs::msg::PoseStamped currentRobotPose() const;

  static double remainingDistance(
    const nav_msgs::msg::Path & path, double x, double y);

  std::string global_frame_;
  std::string robot_frame_;
  std::string action_name_;
  std::string planner_id_;
  std::string goal_topic_;
  std::string plan_path_topic_;
  std::string remaining_distance_topic_;
  double tf_timeout_{0.2};
  double publish_rate_{10.0};
  double action_server_wait_timeout_{2.0};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr remaining_distance_pub_;
  rclcpp::TimerBase::SharedPtr remaining_distance_timer_;
  ActionClient::SharedPtr action_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  std::uint64_t goal_generation_{0};
  GoalHandle::SharedPtr active_goal_handle_;
  nav_msgs::msg::Path active_path_;
};

}  // namespace ackermann_smac_bridge

#endif  // ACKERMANN_SMAC_BRIDGE__ACKERMANN_SMAC_BRIDGE_HPP_
