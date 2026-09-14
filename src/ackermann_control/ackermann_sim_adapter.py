#!/usr/bin/env python3
"""Adapt the canonical Ackermann command to the simulation controller input.

Input (default: /ackermann_cmd):
  - linear.x: longitudinal speed in m/s
  - angular.z: front-wheel steering angle in radians

Output (default: /ackermann_steering_controller/reference):
  geometry_msgs/TwistStamped where angular.z is body yaw rate in rad/s, as
  expected by the ros2_control Ackermann steering controller.
"""

import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class AckermannSimulationAdapter(Node):
    """Convert ``(speed, steering angle)`` into ``(speed, yaw rate)``."""

    def __init__(self) -> None:
        super().__init__('ackermann_sim_adapter')

        self.declare_parameter('input_topic', '/ackermann_cmd')
        self.declare_parameter(
            'output_topic', '/ackermann_steering_controller/reference'
        )
        self.declare_parameter('wheelbase', 0.593)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        if self.wheelbase <= 0.0:
            raise ValueError('wheelbase must be greater than zero')

        self.publisher = self.create_publisher(
            TwistStamped, self.output_topic, 10
        )
        self.subscription = self.create_subscription(
            Twist, self.input_topic, self.command_callback, 10
        )

        self.get_logger().info(
            f'Simulation Ackermann adapter: {self.input_topic} -> '
            f'{self.output_topic}, wheelbase={self.wheelbase:.3f} m'
        )

    def command_callback(self, msg: Twist) -> None:
        speed = float(msg.linear.x)
        steering = float(msg.angular.z)

        output = TwistStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'base_link'

        if not math.isfinite(speed) or not math.isfinite(steering):
            self.get_logger().error(
                'Received non-finite Ackermann command; publishing stop'
            )
            self.publisher.publish(output)
            return

        yaw_rate = speed * math.tan(steering) / self.wheelbase
        if not math.isfinite(yaw_rate):
            self.get_logger().error(
                'Converted yaw rate is non-finite; publishing stop'
            )
            self.publisher.publish(output)
            return

        output.twist.linear.x = speed
        output.twist.angular.z = yaw_rate
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AckermannSimulationAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
