#!/usr/bin/env python3

"""Convert NeuPAN Ackermann steering commands to body yaw-rate commands."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class NeuPANAckermannAdapter(Node):
    """Translate ``(v, steering_angle)`` into ``(v, yaw_rate)``."""

    def __init__(self) -> None:
        super().__init__('neupan_ackermann_adapter')

        self.declare_parameter('input_topic', '/neupan_cmd_vel_raw')
        self.declare_parameter('output_topic', '/neupan_cmd_vel')
        self.declare_parameter('wheelbase', 0.593)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        if self.wheelbase <= 0.0:
            raise ValueError('wheelbase must be greater than zero')

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Twist,
            self.input_topic,
            self.command_callback,
            10,
        )

        self.get_logger().info(
            f'NeuPAN Ackermann adapter: {self.input_topic} -> '
            f'{self.output_topic}, wheelbase={self.wheelbase:.3f}m'
        )

    def command_callback(self, msg: Twist) -> None:
        """Convert one NeuPAN command without changing its linear speed."""
        speed = float(msg.linear.x)
        steering = float(msg.angular.z)

        if not math.isfinite(speed) or not math.isfinite(steering):
            self.get_logger().error(
                'Received non-finite NeuPAN command; publishing zero command'
            )
            self.publisher.publish(Twist())
            return

        yaw_rate = speed * math.tan(steering) / self.wheelbase
        if not math.isfinite(yaw_rate):
            self.get_logger().error(
                'Converted yaw rate is non-finite; publishing zero command'
            )
            self.publisher.publish(Twist())
            return

        output = Twist()
        output.linear.x = speed
        output.angular.z = yaw_rate
        self.publisher.publish(output)


def main(args=None) -> None:
    """Run the Ackermann command adapter."""
    rclpy.init(args=args)
    node = None
    try:
        node = NeuPANAckermannAdapter()
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
