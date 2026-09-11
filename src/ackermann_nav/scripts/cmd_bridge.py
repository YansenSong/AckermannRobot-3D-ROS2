#!/usr/bin/env python3
"""Forward-only Twist to the AckermannRobot TwistStamped controller reference.

The Nav2 controller and velocity smoother publish a body command where
``angular.z`` is yaw rate.  The ros2_control Ackermann controller consumes the
same body-twist semantics, so this bridge deliberately does not convert yaw
rate to a steering angle.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CommandBridge(Node):
    def __init__(self):
        super().__init__("ackermann_cmd_bridge")

        self.declare_parameter("input_topic", "/ackermann_nav/cmd_vel_smoothed")
        self.declare_parameter(
            "output_topic", "/ackermann_steering_controller/reference")
        self.declare_parameter("max_speed", 0.70)
        self.declare_parameter("min_speed", 0.0)
        self.declare_parameter("min_turning_radius", 1.320)
        self.declare_parameter("command_timeout", 0.35)
        self.declare_parameter("output_rate", 30.0)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("min_turning_speed", 0.02)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.max_speed = self._positive_or_zero(
            "max_speed", self.get_parameter("max_speed").value)
        self.min_speed = self._positive_or_zero(
            "min_speed", self.get_parameter("min_speed").value)
        self.min_turning_radius = self._positive(
            "min_turning_radius", self.get_parameter("min_turning_radius").value)
        self.command_timeout = self._positive(
            "command_timeout", self.get_parameter("command_timeout").value)
        self.output_rate = self._positive(
            "output_rate", self.get_parameter("output_rate").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.min_turning_speed = self._positive(
            "min_turning_speed", self.get_parameter("min_turning_speed").value)

        if self.min_speed > self.max_speed:
            raise ValueError("min_speed must not exceed max_speed")

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_input_time = None
        self.last_invalid_warning = 0.0
        self.last_negative_warning = 0.0

        self.publisher = self.create_publisher(TwistStamped, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Twist, self.input_topic, self._on_command, 10)
        self.timer = self.create_timer(1.0 / self.output_rate, self._publish_command)

        self.get_logger().info(
            "Ackermann command bridge: %s -> %s; forward-only, "
            "max_speed=%.2f m/s, Rmin=%.3f m, timeout=%.2f s"
            % (
                self.input_topic,
                self.output_topic,
                self.max_speed,
                self.min_turning_radius,
                self.command_timeout,
            )
        )

    @staticmethod
    def _positive(name, value):
        number = float(value)
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError("%s must be a positive finite number" % name)
        return number

    @staticmethod
    def _positive_or_zero(name, value):
        number = float(value)
        if not math.isfinite(number) or number < 0.0:
            raise ValueError("%s must be a non-negative finite number" % name)
        return number

    def _warn_invalid(self):
        now = time.monotonic()
        if now - self.last_invalid_warning >= 5.0:
            self.get_logger().warning(
                "Received a non-finite Ackermann Nav2 command; publishing zero")
            self.last_invalid_warning = now

    def _warn_negative(self):
        now = time.monotonic()
        if now - self.last_negative_warning >= 5.0:
            self.get_logger().warning(
                "Received a reverse command; rejecting it in forward-only mode")
            self.last_negative_warning = now

    def _on_command(self, msg):
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        self.last_input_time = self.get_clock().now()

        if not math.isfinite(linear) or not math.isfinite(angular):
            self._warn_invalid()
            self.target_linear = 0.0
            self.target_angular = 0.0
            return

        if linear < 0.0:
            self._warn_negative()
            self.target_linear = 0.0
            self.target_angular = 0.0
            return

        self.target_linear = min(max(linear, self.min_speed), self.max_speed)
        self.target_angular = angular

    def _input_is_fresh(self):
        if self.last_input_time is None:
            return False
        age = (self.get_clock().now() - self.last_input_time).nanoseconds * 1.0e-9
        return 0.0 <= age <= self.command_timeout

    def _safe_command(self):
        if not self._input_is_fresh():
            return 0.0, 0.0

        linear = min(max(self.target_linear, self.min_speed), self.max_speed)
        if linear < self.min_turning_speed:
            return linear, 0.0

        max_yaw_rate = linear / self.min_turning_radius
        angular = max(-max_yaw_rate, min(max_yaw_rate, self.target_angular))
        return linear, angular

    def _publish_command(self):
        linear, angular = self._safe_command()
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = self.frame_id
        command.twist.linear.x = linear
        command.twist.angular.z = angular
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = CommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
