#!/usr/bin/env python3
import math
import os
import re
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


G0 = 9.80665
POSES = (
    ("+X", 0, 1.0, (1.0, 0.0, 0.0)),
    ("-X", 0, -1.0, (-1.0, 0.0, 0.0)),
    ("+Y", 1, 1.0, (0.0, 1.0, 0.0)),
    ("-Y", 1, -1.0, (0.0, -1.0, 0.0)),
    ("+Z", 2, 1.0, (0.0, 0.0, 1.0)),
    ("-Z", 2, -1.0, (0.0, 0.0, -1.0)),
)


def statistics(values):
    count = len(values)
    mean = sum(values) / count
    variance = sum((value - mean) ** 2 for value in values) / count
    return {
        "mean": mean,
        "stddev": math.sqrt(variance),
        "rms": math.sqrt(sum(value * value for value in values) / count),
        "min": min(values),
        "max": max(values),
    }


def orientation_error_deg(vector, target):
    norm = math.sqrt(sum(value * value for value in vector))
    if norm <= 1e-12:
        return 180.0
    dot = sum(value * expected for value, expected in zip(vector, target)) / norm
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def solve_axis_calibration(pose_results, gravity=G0):
    offsets = []
    gains = []
    for axis, plus_name, minus_name in (
        (0, "+X", "-X"),
        (1, "+Y", "-Y"),
        (2, "+Z", "-Z"),
    ):
        plus = pose_results[plus_name]["accel_stats"][axis]["mean"]
        minus = pose_results[minus_name]["accel_stats"][axis]["mean"]
        span = plus - minus
        if plus <= minus or span < gravity:
            raise ValueError(
                f"Invalid {plus_name} / {minus_name} measurements: "
                f"{plus:+.6f}, {minus:+.6f} m/s^2"
            )
        offsets.append((plus + minus) / 2.0)
        gains.append((2.0 * gravity) / span)
    return tuple(offsets), tuple(gains)


def corrected_vector(vector, offsets, gains):
    return tuple(
        (value - offset) * gain
        for value, offset, gain in zip(vector, offsets, gains)
    )


def read_numeric_ros_parameters(path):
    """Read scalar numeric parameters from a simple ROS 2 parameter YAML."""
    if not path.is_file():
        return {}
    parameters = {}
    pattern = re.compile(
        r"^\s{4}([A-Za-z_][A-Za-z0-9_]*):\s*"
        r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*$"
    )
    for line in path.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)
        if match:
            parameters[match.group(1)] = float(match.group(2))
    return parameters


class AccelCalibration(Node):
    def __init__(self):
        super().__init__("accel_calibration")
        default_dir = Path.home() / ".ros"

        self.declare_parameter("input_topic", "/imu/data_raw")
        self.declare_parameter("sample_duration", 8.0)
        self.declare_parameter("minimum_samples", 100)
        self.declare_parameter("max_stationary_gyro", 0.03)
        self.declare_parameter("max_motion_ratio", 0.05)
        self.declare_parameter("max_orientation_error_deg", 12.0)
        self.declare_parameter("gravity_m_s2", G0)
        self.declare_parameter("gravity_tolerance", 1.0)
        self.declare_parameter(
            "output_file", str(default_dir / "lpms_ig1_calibration.yaml")
        )
        self.declare_parameter(
            "report_file",
            str(default_dir / "lpms_ig1_accel_calibration_report.yaml"),
        )
        self.declare_parameter("message_timeout", 5.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.sample_duration = float(self.get_parameter("sample_duration").value)
        self.minimum_samples = int(self.get_parameter("minimum_samples").value)
        self.max_stationary_gyro = float(
            self.get_parameter("max_stationary_gyro").value
        )
        self.max_motion_ratio = float(
            self.get_parameter("max_motion_ratio").value
        )
        self.max_orientation_error_deg = float(
            self.get_parameter("max_orientation_error_deg").value
        )
        self.gravity = float(self.get_parameter("gravity_m_s2").value)
        self.gravity_tolerance = float(
            self.get_parameter("gravity_tolerance").value
        )
        self.output_file = Path(
            str(self.get_parameter("output_file").value)
        ).expanduser()
        self.report_file = Path(
            str(self.get_parameter("report_file").value)
        ).expanduser()
        self.message_timeout = float(self.get_parameter("message_timeout").value)
        self._validate_parameters()

        self.collecting = False
        self.samples = []
        self.subscription = self.create_subscription(
            Imu, self.input_topic, self.imu_callback, qos_profile_sensor_data
        )

    def _validate_parameters(self):
        if self.sample_duration <= 0.0:
            raise ValueError("sample_duration must be > 0")
        if self.minimum_samples <= 0:
            raise ValueError("minimum_samples must be > 0")
        if self.max_stationary_gyro <= 0.0:
            raise ValueError("max_stationary_gyro must be > 0")
        if not 0.0 <= self.max_motion_ratio <= 1.0:
            raise ValueError("max_motion_ratio must be in [0, 1]")
        if not 0.0 < self.max_orientation_error_deg < 90.0:
            raise ValueError("max_orientation_error_deg must be in (0, 90)")
        if self.gravity <= 0.0 or self.gravity_tolerance <= 0.0:
            raise ValueError("gravity and gravity_tolerance must be > 0")
        if self.message_timeout <= 0.0:
            raise ValueError("message_timeout must be > 0")

    def imu_callback(self, msg):
        if not self.collecting:
            return
        values = (
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z),
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        )
        if all(math.isfinite(value) for value in values):
            self.samples.append((time.monotonic(), values))

    def collect_pose(self, pose):
        name, _axis, _sign, target = pose
        self.samples = []
        self.collecting = True
        began = time.monotonic()
        first_deadline = began + self.message_timeout
        sampling_deadline = None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            if self.samples and sampling_deadline is None:
                sampling_deadline = self.samples[0][0] + self.sample_duration
            if not self.samples and now >= first_deadline:
                self.collecting = False
                raise RuntimeError(
                    f"No IMU messages received on {self.input_topic}"
                )
            if sampling_deadline is not None and now >= sampling_deadline:
                break

        self.collecting = False
        if not self.samples:
            raise RuntimeError("ROS shutdown before any samples were received")

        timestamps = [sample[0] for sample in self.samples]
        channels = [
            [sample[1][index] for sample in self.samples]
            for index in range(6)
        ]
        count = len(self.samples)
        duration = max(timestamps[-1] - timestamps[0], 1e-9)
        accel_stats = [statistics(channel) for channel in channels[:3]]
        gyro_stats = [statistics(channel) for channel in channels[3:]]
        mean_accel = tuple(item["mean"] for item in accel_stats)
        accel_norms = [
            math.sqrt(ax * ax + ay * ay + az * az)
            for ax, ay, az in zip(*channels[:3])
        ]
        gyro_norms = [
            math.sqrt(gx * gx + gy * gy + gz * gz)
            for gx, gy, gz in zip(*channels[3:])
        ]
        motion_count = sum(
            norm > self.max_stationary_gyro for norm in gyro_norms
        )
        motion_ratio = motion_count / count
        mean_accel_norm = sum(accel_norms) / count
        angle_error = orientation_error_deg(mean_accel, target)

        result = {
            "name": name,
            "samples": count,
            "duration": duration,
            "sample_rate": count / duration,
            "accel_stats": accel_stats,
            "gyro_stats": gyro_stats,
            "mean_accel_norm": mean_accel_norm,
            "orientation_error_deg": angle_error,
            "motion_samples": motion_count,
            "motion_ratio": motion_ratio,
        }
        return result, self.pose_failure(result)

    def pose_failure(self, result):
        if result["samples"] < self.minimum_samples:
            return (
                f"Too few samples ({result['samples']}); "
                f"at least {self.minimum_samples} required."
            )
        if result["motion_ratio"] > self.max_motion_ratio:
            return (
                "IMU moved during sampling: motion ratio "
                f"{result['motion_ratio']:.1%}, maximum "
                f"{self.max_motion_ratio:.1%}."
            )
        if result["orientation_error_deg"] > self.max_orientation_error_deg:
            return (
                f"Requested {result['name']} UP, measured direction error "
                f"{result['orientation_error_deg']:.2f} deg; maximum "
                f"{self.max_orientation_error_deg:.2f} deg."
            )
        gravity_error = abs(result["mean_accel_norm"] - self.gravity)
        if gravity_error > self.gravity_tolerance:
            return (
                "Acceleration magnitude is inconsistent with gravity: "
                f"{result['mean_accel_norm']:.4f} m/s^2, expected "
                f"{self.gravity:.4f} +/- {self.gravity_tolerance:.4f} m/s^2."
            )
        return None

    def print_pose_result(self, result):
        means = [item["mean"] for item in result["accel_stats"]]
        self.get_logger().info("PASS")
        self.get_logger().info(
            f"Samples: {result['samples']}, duration: {result['duration']:.2f} s, "
            f"rate: {result['sample_rate']:.1f} Hz"
        )
        self.get_logger().info(
            "Mean acceleration [m/s^2]: "
            f"X={means[0]:+.5f}, Y={means[1]:+.5f}, Z={means[2]:+.5f}"
        )
        self.get_logger().info(
            f"Mean |a|: {result['mean_accel_norm']:.5f} m/s^2, "
            f"orientation error: {result['orientation_error_deg']:.2f} deg"
        )

    def run(self):
        self.print_intro()
        results = {}
        for step, pose in enumerate(POSES, start=1):
            while rclpy.ok():
                name = pose[0]
                print(f"\nSTEP {step} / 6\n")
                print(f"Place the IMU with {name} pointing upward.")
                print("Keep the sensor completely stationary.")
                input("Press ENTER when ready. ")
                self.get_logger().info("Checking orientation...")
                self.get_logger().info(
                    f"Collecting data: {self.sample_duration:.1f} seconds"
                )
                result, failure = self.collect_pose(pose)
                if failure is None:
                    results[name] = result
                    self.print_pose_result(result)
                    break
                self.get_logger().error(f"{name} sampling FAILED: {failure}")
                self.get_logger().info(
                    "Reposition the IMU and retry this orientation."
                )

        if len(results) != len(POSES):
            raise RuntimeError("ROS shutdown before all orientations completed")
        offsets, gains = solve_axis_calibration(results, self.gravity)
        self.check_solution(offsets, gains)
        validation, maximum_off_axis = self.validate_results(
            results, offsets, gains
        )
        self.print_final(offsets, gains, validation, maximum_off_axis)
        self.write_calibration(offsets, gains)
        self.write_report(results, offsets, gains, validation, maximum_off_axis)
        self.get_logger().info(f"Calibration parameters: {self.output_file}")
        self.get_logger().info(f"Detailed report: {self.report_file}")

    def print_intro(self):
        print("=" * 60)
        print("LPMS-IG1 SIX-POSITION ACCELEROMETER CALIBRATION")
        print("=" * 60)
        print("\nThis procedure requires six stationary orientations.\n")
        for index, pose in enumerate(POSES, start=1):
            print(f"{index}. {pose[0]} UP")

    def check_solution(self, offsets, gains):
        for axis, (offset, gain) in enumerate(zip(offsets, gains)):
            name = "XYZ"[axis]
            if not 0.5 < gain < 1.5:
                raise ValueError(
                    f"Calibration FAILED: implausible {name} gain {gain:.6f}"
                )
            if not 0.8 < gain < 1.2:
                self.get_logger().warning(
                    f"Unexpected accelerometer gain on {name}: {gain:.6f}"
                )
            if abs(offset) > 2.0:
                self.get_logger().warning(
                    f"Large accelerometer offset on {name}: "
                    f"{offset:+.6f} m/s^2"
                )

    def validate_results(self, results, offsets, gains):
        validation = {}
        maximum_off_axis = 0.0
        for name, axis, _sign, _target in POSES:
            mean = tuple(
                item["mean"] for item in results[name]["accel_stats"]
            )
            corrected = corrected_vector(mean, offsets, gains)
            norm = math.sqrt(sum(value * value for value in corrected))
            off_axis = max(
                abs(value) for index, value in enumerate(corrected)
                if index != axis
            )
            maximum_off_axis = max(maximum_off_axis, off_axis)
            validation[name] = {"corrected": corrected, "norm": norm}
        if maximum_off_axis > 0.5:
            self.get_logger().warning(
                "Maximum off-axis residual is high: "
                f"{maximum_off_axis:.4f} m/s^2"
            )
        return validation, maximum_off_axis

    def print_final(self, offsets, gains, validation, maximum_off_axis):
        lines = [
            "=" * 60,
            "VALIDATION",
            "=" * 60,
            "Pose       X          Y          Z        |a|",
            "-------------------------------------------------------",
        ]
        for name, _axis, _sign, _target in POSES:
            vector = validation[name]["corrected"]
            lines.append(
                f"{name:>3}  {vector[0]:+10.4f} {vector[1]:+10.4f} "
                f"{vector[2]:+10.4f} {validation[name]['norm']:9.4f}"
            )
        lines.extend([
            "=" * 60,
            "LPMS-IG1 ACCELEROMETER CALIBRATION RESULT",
            "=" * 60,
            "Offset [m/s^2]",
            f"X: {offsets[0]:+.8f}",
            f"Y: {offsets[1]:+.8f}",
            f"Z: {offsets[2]:+.8f}",
            "Correction gain",
            f"X: {gains[0]:.8f}",
            f"Y: {gains[1]:.8f}",
            f"Z: {gains[2]:.8f}",
            f"Maximum off-axis residual: {maximum_off_axis:.6f} m/s^2",
            "Calibration status: PASS",
            "=" * 60,
        ])
        for line in lines:
            self.get_logger().info(line)

    def write_calibration(self, offsets, gains):
        existing = read_numeric_ros_parameters(self.output_file)
        legacy_gyro_file = Path.home() / ".ros" / "lpms_ig1_gyro_calibration.yaml"
        if not self.output_file.exists():
            existing.update(read_numeric_ros_parameters(legacy_gyro_file))
        gyro = [existing.get(f"gyro_bias_{axis}", 0.0) for axis in "xyz"]
        content = ["lpms_ig1_node:", "  ros__parameters:"]
        for axis, value in zip("xyz", gyro):
            content.append(f"    gyro_bias_{axis}: {value:.12g}")
        for axis, value in zip("xyz", offsets):
            content.append(f"    accel_offset_{axis}: {value:.12g}")
        for axis, value in zip("xyz", gains):
            content.append(f"    accel_gain_{axis}: {value:.12g}")
        self.atomic_write(self.output_file, "\n".join(content) + "\n")

    def write_report(
        self, results, offsets, gains, validation, maximum_off_axis
    ):
        lines = [
            f"timestamp_utc: '{datetime.now(timezone.utc).isoformat()}'",
            f"gravity_m_s2: {self.gravity:.12g}",
            "poses:",
        ]
        for name, _axis, _sign, _target in POSES:
            result = results[name]
            means = [item["mean"] for item in result["accel_stats"]]
            stddevs = [item["stddev"] for item in result["accel_stats"]]
            lines.extend([
                f"  '{name}':",
                f"    samples: {result['samples']}",
                f"    duration_sec: {result['duration']:.9g}",
                f"    sample_rate_hz: {result['sample_rate']:.9g}",
                "    mean_accel_m_s2: "
                f"[{means[0]:.12g}, {means[1]:.12g}, {means[2]:.12g}]",
                "    stddev_accel_m_s2: "
                f"[{stddevs[0]:.12g}, {stddevs[1]:.12g}, {stddevs[2]:.12g}]",
                f"    mean_accel_norm_m_s2: {result['mean_accel_norm']:.12g}",
                f"    orientation_error_deg: {result['orientation_error_deg']:.9g}",
                f"    motion_ratio: {result['motion_ratio']:.9g}",
            ])
        lines.extend([
            "result:",
            "  accel_offset_m_s2: "
            f"[{offsets[0]:.12g}, {offsets[1]:.12g}, {offsets[2]:.12g}]",
            "  accel_gain: "
            f"[{gains[0]:.12g}, {gains[1]:.12g}, {gains[2]:.12g}]",
            f"  maximum_off_axis_residual_m_s2: {maximum_off_axis:.12g}",
            "  validation:",
        ])
        for name, _axis, _sign, _target in POSES:
            vector = validation[name]["corrected"]
            lines.append(
                f"    '{name}': [{vector[0]:.12g}, {vector[1]:.12g}, "
                f"{vector[2]:.12g}]"
            )
        self.atomic_write(self.report_file, "\n".join(lines) + "\n")

    @staticmethod
    def atomic_write(path, content):
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + ".tmp")
        temporary.write_text(content, encoding="utf-8")
        os.replace(temporary, path)


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = AccelCalibration()
        node.run()
        exit_code = 0
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info(
                "Accelerometer calibration cancelled by user."
            )
    except (EOFError, OSError, RuntimeError, ValueError) as exc:
        if node is not None:
            node.get_logger().error(f"Calibration FAILED: {exc}")
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
