#!/usr/bin/env python3
"""Connect only the isolated Nav2 command topic to the vehicle controller."""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool


class Nav2CommandAdapter(Node):
    """Convert Nav2 Twist commands to the controller's stamped interface."""

    def __init__(self):
        super().__init__('nav2_cmd_adapter')
        self.declare_parameter('input_topic', '/nav2/cmd_vel')
        self.declare_parameter(
            'output_topic', '/ackermann_steering_controller/reference')
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('publish_rate', 20.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        if self.command_timeout <= 0.0:
            raise ValueError('command_timeout must be greater than zero')
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')

        self.publisher = self.create_publisher(TwistStamped, output_topic, 10)
        self.create_subscription(Twist, input_topic, self.command_callback, 10)
        self.create_subscription(Bool, '/stop', self.stop_callback, 10)
        self.latest_command = None
        self.latest_command_time = None
        self.stop_requested = False
        self.sent_timeout_stop = False
        self.create_timer(1.0 / publish_rate, self.publish_command)

        self.get_logger().info(
            f'Nav2-only command path: {input_topic} -> {output_topic}')

    def command_callback(self, message):
        self.latest_command = message
        self.latest_command_time = self.get_clock().now()
        self.sent_timeout_stop = False

    def stop_callback(self, message):
        self.stop_requested = bool(message.data)

    def publish_command(self):
        now = self.get_clock().now()
        command_is_fresh = (
            self.latest_command is not None
            and self.latest_command_time is not None
            and (now - self.latest_command_time).nanoseconds * 1e-9
            <= self.command_timeout
        )

        if self.stop_requested or not command_is_fresh:
            if self.sent_timeout_stop and not self.stop_requested:
                return
            command = Twist()
            self.sent_timeout_stop = True
        else:
            command = self.latest_command

        stamped = TwistStamped()
        stamped.header.stamp = now.to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist = command
        self.publisher.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2CommandAdapter()
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
