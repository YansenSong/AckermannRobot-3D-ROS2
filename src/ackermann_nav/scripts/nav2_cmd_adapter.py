#!/usr/bin/env python3
"""Convert Nav2 body-twist commands to the project's Ackermann command contract.

Input Twist semantics:
  linear.x  = longitudinal speed [m/s]
  angular.z = vehicle yaw rate [rad/s]

Output Twist semantics:
  linear.x  = longitudinal speed [m/s]
  angular.z = equivalent front-wheel steering angle [rad]
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Nav2CmdAdapter(Node):
    def __init__(self):
        super().__init__('ackermann_nav_cmd_adapter')

        self.declare_parameter('input_topic', '/ackermann_nav/cmd_vel_smoothed')
        self.declare_parameter('output_topic', '/ackermann_nav/ackermann_cmd_raw')
        self.declare_parameter('wheelbase', 0.0)
        self.declare_parameter('max_speed', 0.0)
        self.declare_parameter('min_turning_radius', 0.0)
        self.declare_parameter('command_timeout', 0.35)
        self.declare_parameter('output_rate', 30.0)
        self.declare_parameter('min_turning_speed', 0.02)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.wheelbase = self._positive('wheelbase', self.get_parameter('wheelbase').value)
        self.max_speed = self._positive('max_speed', self.get_parameter('max_speed').value)
        self.min_turning_radius = self._positive(
            'min_turning_radius', self.get_parameter('min_turning_radius').value
        )
        self.command_timeout = self._positive(
            'command_timeout', self.get_parameter('command_timeout').value
        )
        self.output_rate = self._positive(
            'output_rate', self.get_parameter('output_rate').value
        )
        self.min_turning_speed = self._positive(
            'min_turning_speed', self.get_parameter('min_turning_speed').value
        )
        self.max_steering_angle = math.atan(
            self.wheelbase / self.min_turning_radius
        )

        self.target_speed = 0.0
        self.target_yaw_rate = 0.0
        self.last_input_time = None
        self.last_warning_time = 0.0

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Twist, self.input_topic, self._on_command, 10
        )
        self.timer = self.create_timer(1.0 / self.output_rate, self._publish_command)

        self.get_logger().info(
            'Nav2 Ackermann adapter: %s -> %s; L=%.3f m, Rmin=%.3f m, '
            'delta_max=%.3f rad'
            % (
                self.input_topic,
                self.output_topic,
                self.wheelbase,
                self.min_turning_radius,
                self.max_steering_angle,
            )
        )

    @staticmethod
    def _positive(name, value):
        number = float(value)
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError('%s must be a positive finite number' % name)
        return number

    def _warn(self, text):
        now = time.monotonic()
        if now - self.last_warning_time >= 5.0:
            self.get_logger().warning(text)
            self.last_warning_time = now

    def _on_command(self, msg):
        speed = float(msg.linear.x)
        yaw_rate = float(msg.angular.z)
        self.last_input_time = self.get_clock().now()

        if not math.isfinite(speed) or not math.isfinite(yaw_rate):
            self._warn('Received non-finite Nav2 command; replacing it with stop')
            self.target_speed = 0.0
            self.target_yaw_rate = 0.0
            return

        if speed < 0.0:
            self._warn('Received reverse Nav2 command; this stack is forward-only')
            self.target_speed = 0.0
            self.target_yaw_rate = 0.0
            return

        self.target_speed = min(speed, self.max_speed)
        self.target_yaw_rate = yaw_rate

    def _input_is_fresh(self):
        if self.last_input_time is None:
            return False
        age = (self.get_clock().now() - self.last_input_time).nanoseconds * 1.0e-9
        return 0.0 <= age <= self.command_timeout

    def _safe_command(self):
        if not self._input_is_fresh():
            return 0.0, 0.0

        speed = min(max(self.target_speed, 0.0), self.max_speed)
        if speed < self.min_turning_speed:
            return speed, 0.0

        steering = math.atan(self.wheelbase * self.target_yaw_rate / speed)
        steering = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, steering),
        )
        return speed, steering

    def _publish_command(self):
        speed, steering = self._safe_command()
        command = Twist()
        command.linear.x = speed
        command.angular.z = steering
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2CmdAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
