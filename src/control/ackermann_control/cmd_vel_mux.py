#!/usr/bin/env python3
"""
cmd_vel_mux.py — manual/autonomous command selection with a stop override

Prioritizes fresh teleoperation commands from /cmd_vel over NeuPAN commands
from /neupan_cmd_vel, and forwards the selected command to the
ackermann_steering_controller as TwistStamped. The /stop override has highest
priority.

The /stop topic is a centralized stop override.  Publishing
std_msgs/msg/Bool with data=true forces zero velocity regardless of the
currently selected planner; publishing data=false releases the override.

"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        self.pub = self.create_publisher(
            TwistStamped, '/ackermann_steering_controller/reference', 10
        )

        self.neupan_sub = self.create_subscription(
            Twist, '/neupan_cmd_vel', self.neupan_callback, 10
        )
        self.manual_sub = self.create_subscription(
            Twist, '/cmd_vel', self.manual_callback, 10
        )
        self.stop_sub = self.create_subscription(
            Bool, '/stop', self.stop_callback, 10
        )

        self.neupan_msg: Twist | None = None
        self.manual_msg: Twist | None = None
        self.manual_received_at: float | None = None
        self.declare_parameter('manual_cmd_timeout', 0.5)
        self.manual_cmd_timeout = float(
            self.get_parameter('manual_cmd_timeout').value
        )
        self.stop_requested = False

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            'cmd_vel_mux started: manual(/cmd_vel) overrides '
            'NeuPAN(/neupan_cmd_vel); /stop overrides both → '
            '/ackermann_steering_controller/reference'
        )

    def neupan_callback(self, msg: Twist):
        self.neupan_msg = msg

    def manual_callback(self, msg: Twist):
        self.manual_msg = msg
        self.manual_received_at = time.monotonic()

    def stop_callback(self, msg: Bool):
        requested = bool(msg.data)
        if requested != self.stop_requested:
            state = 'ACTIVE' if requested else 'RELEASED'
            self.get_logger().warn(
                f'/stop override {state}' if requested
                else '/stop override released'
            )
        self.stop_requested = requested

    def timer_callback(self):
        if self.stop_requested:
            # Publish at the mux timer rate so the controller receives a
            # continuous zero command and cannot resume from a stale command.
            twist = Twist()
            source = 'stop'
        else:
            manual_is_fresh = (
                self.manual_msg is not None
                and self.manual_received_at is not None
                and time.monotonic() - self.manual_received_at
                <= self.manual_cmd_timeout
            )
            if manual_is_fresh:
                twist = self.manual_msg
                source = 'manual'
            elif self.neupan_msg is not None:
                twist = self.neupan_msg
                source = 'neupan'
            else:
                return

        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'base_link'
        ts.twist = twist
        self.pub.publish(ts)

        self.get_logger().debug(
            f'[{source}] v={twist.linear.x:.2f}, ω={twist.angular.z:.2f}',
            throttle_duration_sec=10.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
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
