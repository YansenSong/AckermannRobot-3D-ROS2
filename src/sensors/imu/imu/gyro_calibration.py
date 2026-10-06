#!/usr/bin/env python3
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


G0 = 9.80665


def axis_statistics(values):
    """Return population statistics for one gyro axis."""
    count = len(values)
    mean = sum(values) / count
    variance = sum((value - mean) ** 2 for value in values) / count
    rms = math.sqrt(sum(value * value for value in values) / count)
    minimum = min(values)
    maximum = max(values)
    return {
        "mean": mean,
        "stddev": math.sqrt(variance),
        "rms": rms,
        "min": minimum,
        "max": maximum,
        "peak_to_peak": maximum - minimum,
    }


class GyroCalibration(Node):
    def __init__(self):
        super().__init__("gyro_calibration")

        self.declare_parameter("input_topic", "/imu/data_raw")
        self.declare_parameter("warmup_duration", 5.0)
        self.declare_parameter("calibration_duration", 30.0)
        self.declare_parameter("max_stationary_gyro", 0.03)
        self.declare_parameter("accel_norm_tolerance", 1.0)
        self.declare_parameter("max_motion_ratio", 0.05)
        self.declare_parameter(
            "output_file",
            str(Path.home() / ".ros" / "lpms_ig1_gyro_calibration.yaml"),
        )
        self.declare_parameter("message_timeout", 5.0)
        self.declare_parameter("min_samples", 100)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.warmup_duration = float(
            self.get_parameter("warmup_duration").value
        )
        self.calibration_duration = float(
            self.get_parameter("calibration_duration").value
        )
        self.max_stationary_gyro = float(
            self.get_parameter("max_stationary_gyro").value
        )
        self.accel_norm_tolerance = float(
            self.get_parameter("accel_norm_tolerance").value
        )
        self.max_motion_ratio = float(
            self.get_parameter("max_motion_ratio").value
        )
        self.output_file = Path(
            str(self.get_parameter("output_file").value)
        ).expanduser()
        self.message_timeout = float(
            self.get_parameter("message_timeout").value
        )
        self.min_samples = int(self.get_parameter("min_samples").value)

        self._validate_parameters()

        self.started_at = time.monotonic()
        self.first_message_at = None
        self.last_message_at = None
        self.calibration_started_at = None
        self.finished = False
        self.exit_code = 1

        self.gyro_samples = [[], [], []]
        self.accel_norms = []
        self.gyro_motion_samples = 0
        self.accel_outlier_samples = 0

        self.subscription = self.create_subscription(
            Imu, self.input_topic, self.imu_callback, qos_profile_sensor_data
        )
        self.watchdog = self.create_timer(0.2, self.check_timeout)

        self.get_logger().info("LPMS-IG1 static gyro calibration")
        self.get_logger().info("Keep the IMU completely stationary.")
        self.get_logger().info(f"Waiting for IMU data on {self.input_topic}...")
        self.get_logger().info(f"Warm-up: {self.warmup_duration:.1f} s")
        self.get_logger().info(
            f"Calibration: {self.calibration_duration:.1f} s"
        )

    def _validate_parameters(self):
        if self.warmup_duration < 0.0:
            raise ValueError("warmup_duration must be >= 0")
        if self.calibration_duration <= 0.0:
            raise ValueError("calibration_duration must be > 0")
        if self.max_stationary_gyro <= 0.0:
            raise ValueError("max_stationary_gyro must be > 0")
        if self.accel_norm_tolerance <= 0.0:
            raise ValueError("accel_norm_tolerance must be > 0")
        if not 0.0 <= self.max_motion_ratio <= 1.0:
            raise ValueError("max_motion_ratio must be in [0, 1]")
        if self.message_timeout <= 0.0:
            raise ValueError("message_timeout must be > 0")
        if self.min_samples <= 0:
            raise ValueError("min_samples must be > 0")
        if not str(self.output_file):
            raise ValueError("output_file must not be empty")

    def imu_callback(self, msg):
        if self.finished:
            return

        now = time.monotonic()
        self.last_message_at = now
        if self.first_message_at is None:
            self.first_message_at = now
            self.get_logger().info("IMU data received; warm-up started.")

        if now - self.first_message_at < self.warmup_duration:
            return

        if self.calibration_started_at is None:
            self.calibration_started_at = now
            self.get_logger().info("Warm-up complete; collecting samples.")

        gx = float(msg.angular_velocity.x)
        gy = float(msg.angular_velocity.y)
        gz = float(msg.angular_velocity.z)
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)

        values = (gx, gy, gz, ax, ay, az)
        if not all(math.isfinite(value) for value in values):
            self.fail("Received non-finite IMU data.")
            return

        self.gyro_samples[0].append(gx)
        self.gyro_samples[1].append(gy)
        self.gyro_samples[2].append(gz)

        gyro_norm = math.sqrt(gx * gx + gy * gy + gz * gz)
        if gyro_norm > self.max_stationary_gyro:
            self.gyro_motion_samples += 1

        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
        self.accel_norms.append(accel_norm)
        if abs(accel_norm - G0) > self.accel_norm_tolerance:
            self.accel_outlier_samples += 1

        if now - self.calibration_started_at >= self.calibration_duration:
            self.complete(now)

    def check_timeout(self):
        if self.finished:
            return
        now = time.monotonic()
        reference = self.last_message_at or self.started_at
        if now - reference <= self.message_timeout:
            return
        if self.first_message_at is None:
            self.fail(f"No IMU messages received on {self.input_topic}")
        else:
            self.fail(f"IMU messages stopped on {self.input_topic}")

    def complete(self, ended_at):
        count = len(self.gyro_samples[0])
        duration = ended_at - self.calibration_started_at
        if count < self.min_samples:
            self.fail(
                f"Too few samples: {count}; at least {self.min_samples} required."
            )
            return

        stats = [axis_statistics(axis) for axis in self.gyro_samples]
        gyro_motion_ratio = self.gyro_motion_samples / count
        accel_outlier_ratio = self.accel_outlier_samples / count
        accel_mean = sum(self.accel_norms) / count

        self.print_report(
            duration,
            count,
            stats,
            gyro_motion_ratio,
            accel_outlier_ratio,
            accel_mean,
        )

        failures = []
        if gyro_motion_ratio > self.max_motion_ratio:
            failures.append(
                "gyro motion ratio "
                f"{gyro_motion_ratio:.1%} exceeds {self.max_motion_ratio:.1%}"
            )
        if accel_outlier_ratio > self.max_motion_ratio:
            failures.append(
                "acceleration outlier ratio "
                f"{accel_outlier_ratio:.1%} exceeds {self.max_motion_ratio:.1%}"
            )
        if failures:
            self.fail(
                "IMU was not stationary during calibration: " + "; ".join(failures)
            )
            return

        try:
            self.write_bias_file(stats)
        except OSError as exc:
            self.fail(f"Could not write calibration file: {exc}")
            return

        self.get_logger().info(f"Calibration status: PASS")
        self.get_logger().info(f"Bias parameters written to {self.output_file}")
        self.exit_code = 0
        self.finished = True

    def print_report(
        self,
        duration,
        count,
        stats,
        gyro_motion_ratio,
        accel_outlier_ratio,
        accel_mean,
    ):
        lines = [
            "=" * 60,
            "LPMS-IG1 STATIC GYRO CALIBRATION RESULT",
            "=" * 60,
            f"Duration            : {duration:.2f} s",
            f"Samples             : {count}",
            f"Sample rate          : {count / duration:.1f} Hz",
            "Gyroscope statistics:",
        ]
        for name, values in zip(("X", "Y", "Z"), stats):
            lines.extend([
                f"{name}:",
                f"  bias               : {values['mean']:+.9f} rad/s",
                "  bias               : "
                f"{math.degrees(values['mean']):+.6f} deg/s",
                f"  stddev             : {values['stddev']:.9f} rad/s",
                f"  RMS                : {values['rms']:.9f} rad/s",
                f"  min                : {values['min']:+.9f} rad/s",
                f"  max                : {values['max']:+.9f} rad/s",
                "  peak-to-peak       : "
                f"{values['peak_to_peak']:.9f} rad/s",
            ])
        lines.extend([
            f"Gyro motion samples : {self.gyro_motion_samples} / {count} "
            f"({gyro_motion_ratio:.1%})",
            f"Accel outliers      : {self.accel_outlier_samples} / {count} "
            f"({accel_outlier_ratio:.1%})",
            f"Accel norm mean     : {accel_mean:.4f} m/s^2",
            "=" * 60,
        ])
        for line in lines:
            self.get_logger().info(line)

    def write_bias_file(self, stats):
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        content = (
            "lpms_ig1_node:\n"
            "  ros__parameters:\n"
            f"    gyro_bias_x: {stats[0]['mean']:.12g}\n"
            f"    gyro_bias_y: {stats[1]['mean']:.12g}\n"
            f"    gyro_bias_z: {stats[2]['mean']:.12g}\n"
        )
        self.output_file.write_text(content, encoding="utf-8")

    def fail(self, reason):
        if self.finished:
            return
        self.get_logger().error("Calibration FAILED.")
        self.get_logger().error(f"Reason: {reason}")
        self.exit_code = 1
        self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = GyroCalibration()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
        exit_code = node.exit_code
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Calibration cancelled by user.")
    except (OSError, ValueError) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"ERROR: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
