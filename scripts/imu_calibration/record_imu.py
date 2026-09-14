#!/usr/bin/env python3
"""Record /imu/data to a CSV for Allan-variance / mount-pose analysis.

Usage (from the ROS workspace, with install/setup.bash sourced):

    python3 scripts/imu_calibration/record_imu.py --duration 2400 --output imu_static.csv

The vehicle must stay stationary and level for the whole recording — the
Allan-variance analysis is only meaningful on static data.
"""

import argparse
import csv
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuRecorder(Node):
    def __init__(self, duration, output):
        super().__init__('imu_recorder')
        self.duration = duration
        self.start = time.monotonic()
        self.deadline = self.start + duration
        self.file = open(output, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'secs', 'nsecs',
            'acc_x', 'acc_y', 'acc_z',
            'gyr_x', 'gyr_y', 'gyr_z',
            'q_w', 'q_x', 'q_y', 'q_z',
        ])
        self.count = 0
        self.last_progress = self.start

        self.sub = self.create_subscription(Imu, '/imu/data', self.cb, 100)
        self.timer = self.create_timer(0.5, self.progress)
        self.get_logger().info(
            f'Recording /imu/data for {duration:.0f}s to {output} — '
            'keep the vehicle perfectly still and level.')

    def cb(self, m):
        if time.monotonic() > self.deadline:
            return
        a = m.linear_acceleration
        g = m.angular_velocity
        q = m.orientation
        self.writer.writerow([
            m.header.stamp.sec, m.header.stamp.nanosec,
            f'{a.x:.9f}', f'{a.y:.9f}', f'{a.z:.9f}',
            f'{g.x:.9f}', f'{g.y:.9f}', f'{g.z:.9f}',
            f'{q.w:.9f}', f'{q.x:.9f}', f'{q.y:.9f}', f'{q.z:.9f}',
        ])
        self.count += 1

    def progress(self):
        now = time.monotonic()
        if now - self.last_progress < 10.0:
            return
        self.last_progress = now
        elapsed = now - self.start
        rate = self.count / elapsed if elapsed > 0 else 0.0
        self.get_logger().info(
            f'[{elapsed:.0f}/{self.duration:.0f}s] samples={self.count} '
            f'rate~{rate:.1f}Hz')
        if now >= self.deadline:
            self.shutdown()

    def shutdown(self):
        elapsed = time.monotonic() - self.start
        self.file.flush()
        self.file.close()
        rate = self.count / elapsed if elapsed > 0 else 0.0
        self.get_logger().info(
            f'Done: {self.count} samples in {elapsed:.0f}s '
            f'(rate~{rate:.1f}Hz) -> file closed.')
        self.get_logger().info('Shutting down.')
        raise SystemExit(0)


def main():
    parser = argparse.ArgumentParser(description='Record /imu/data to CSV')
    parser.add_argument('--duration', type=float, default=2400.0,
                        help='recording duration in seconds (default 2400 = 40 min)')
    parser.add_argument('--output', default='imu_static.csv',
                        help='output CSV path')
    args = parser.parse_args()

    rclpy.init()
    node = ImuRecorder(args.duration, args.output)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
