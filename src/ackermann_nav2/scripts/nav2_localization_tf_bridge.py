#!/usr/bin/env python3
"""Publish a current-time map-to-odom transform from HDL localization."""

import math
import threading

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time


def yaw_from_quaternion(quaternion):
    """Return planar yaw from a quaternion."""
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


class LocalizationTfBridge(Node):
    """Convert timestamped map poses into a continuously valid map/odom TF."""

    def __init__(self):
        super().__init__('nav2_localization_tf_bridge')
        self.declare_parameter('localization_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('transform_tolerance', 0.15)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.transform_tolerance = float(
            self.get_parameter('transform_tolerance').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')
        if self.transform_tolerance < 0.0:
            raise ValueError('transform_tolerance cannot be negative')

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=False)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._lock = threading.Lock()
        self._map_to_odom = None

        self.create_subscription(
            Odometry,
            self.get_parameter('localization_topic').value,
            self.localization_callback,
            10,
        )
        self.create_timer(1.0 / publish_rate, self.publish_transform)
        self.get_logger().info(
            'Waiting for HDL map pose and odom->base_link TF')

    def localization_callback(self, message):
        stamp = Time.from_msg(message.header.stamp)
        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                stamp,
                timeout=Duration(seconds=0.1),
            )
        except tf2_ros.TransformException as error:
            self.get_logger().warning(
                f'Cannot align HDL pose with odometry: {error}',
                throttle_duration_sec=2.0,
            )
            return

        map_pose = message.pose.pose
        map_base_yaw = yaw_from_quaternion(map_pose.orientation)
        odom_base_yaw = yaw_from_quaternion(odom_to_base.transform.rotation)
        map_odom_yaw = map_base_yaw - odom_base_yaw

        odom_x = odom_to_base.transform.translation.x
        odom_y = odom_to_base.transform.translation.y
        cos_yaw = math.cos(map_odom_yaw)
        sin_yaw = math.sin(map_odom_yaw)
        map_odom_x = map_pose.position.x - (cos_yaw * odom_x - sin_yaw * odom_y)
        map_odom_y = map_pose.position.y - (sin_yaw * odom_x + cos_yaw * odom_y)

        with self._lock:
            self._map_to_odom = (map_odom_x, map_odom_y, map_odom_yaw)

    def publish_transform(self):
        with self._lock:
            transform = self._map_to_odom
        if transform is None:
            return

        x, y, yaw = transform
        output = TransformStamped()
        output.header.stamp = (
            self.get_clock().now()
            + Duration(seconds=self.transform_tolerance)
        ).to_msg()
        output.header.frame_id = self.map_frame
        output.child_frame_id = self.odom_frame
        output.transform.translation.x = x
        output.transform.translation.y = y
        output.transform.rotation.z = math.sin(yaw * 0.5)
        output.transform.rotation.w = math.cos(yaw * 0.5)
        self.tf_broadcaster.sendTransform(output)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationTfBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
