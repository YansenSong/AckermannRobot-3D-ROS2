#!/usr/bin/env python3
"""
cmd_vel_mux.py — NeuPAN command gate with a stop override

Forwards NeuPAN (/neupan_cmd_vel) to the ackermann_steering_controller as
TwistStamped, unless the /stop override is active.

The /stop topic is a centralized stop override.  Publishing
std_msgs/msg/Bool with data=true forces zero velocity regardless of the
currently selected planner; publishing data=false releases the override.

"""
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
        self.stop_sub = self.create_subscription(
            Bool, '/stop', self.stop_callback, 10
        )

        self.neupan_msg: Twist | None = None
        self.stop_requested = False

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            'cmd_vel_mux started: NeuPAN(/neupan_cmd_vel) + /stop '
            '→ /ackermann_steering_controller/reference'
        )

    def neupan_callback(self, msg: Twist):
        self.neupan_msg = msg

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
            if self.neupan_msg is None:
                return
            twist = self.neupan_msg
            source = 'neupan'

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
