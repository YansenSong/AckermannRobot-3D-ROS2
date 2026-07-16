#!/usr/bin/env python3
"""
cmd_vel_bridge.py
将 Nav2 控制器发布的 /cmd_vel (Twist) 转换为
ackermann_steering_controller 需要的 /ackermann_steering_controller/reference (TwistStamped)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # 订阅 Nav2 controller_server 发布的 /cmd_vel
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # 发布到 ackermann_steering_controller
        self.pub = self.create_publisher(
            TwistStamped, '/ackermann_steering_controller/reference', 10
        )

        self.get_logger().info('cmd_vel_bridge started: /cmd_vel → /ackermann_steering_controller/reference')

    def cmd_vel_callback(self, msg: Twist):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'base_link'
        ts.twist = msg
        self.pub.publish(ts)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
