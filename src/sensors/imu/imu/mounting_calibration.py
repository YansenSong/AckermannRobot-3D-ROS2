#!/usr/bin/env python3
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

G0 = 9.80665


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
            (-sp, cp*sr, cp*cr))


def transpose(matrix):
    return tuple(tuple(matrix[r][c] for r in range(3)) for c in range(3))


def rotate_vector(matrix, vector):
    return tuple(sum(matrix[r][c]*vector[c] for c in range(3)) for r in range(3))


def estimate_mounting_rp(acceleration):
    ax, ay, az = acceleration
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm <= 1e-12:
        raise ValueError("acceleration norm is zero")
    nx, ny, nz = ax/norm, ay/norm, az/norm
    return math.atan2(ny, nz), math.atan2(-nx, math.sqrt(ny*ny + nz*nz))


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


class MountingCalibration(Node):
    def __init__(self):
        super().__init__("mounting_calibration")
        defaults = {
            "input_topic": "/imu/data", "warmup_duration": 5.0,
            "sample_duration": 30.0, "minimum_samples": 100,
            "max_stationary_gyro": 0.03, "max_motion_ratio": 0.05,
            "gravity_m_s2": G0, "gravity_tolerance": 0.5,
            "max_horizontal_residual": 0.30, "mount_yaw_deg": 0.0,
            "mount_x": 0.0, "mount_y": 0.0, "mount_z": 0.0,
            "parent_frame": "base_link", "child_frame": "imu_link",
            "output_file": str(Path.home()/".ros"/"lpms_ig1_mounting.yaml"),
            "message_timeout": 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self.p["output_file"] = Path(str(self.p["output_file"])).expanduser()
        self.yaw = math.radians(float(self.p["mount_yaw_deg"]))
        self.translation = tuple(float(self.p[f"mount_{a}"]) for a in "xyz")
        if float(self.p["sample_duration"]) <= 0 or int(self.p["minimum_samples"]) <= 0:
            raise ValueError("sample_duration and minimum_samples must be positive")
        self.active = False
        self.samples = []
        self.create_subscription(Imu, str(self.p["input_topic"]), self.callback,
                                 qos_profile_sensor_data)

    def callback(self, msg):
        if not self.active:
            return
        values = (msg.linear_acceleration.x, msg.linear_acceleration.y,
                  msg.linear_acceleration.z, msg.angular_velocity.x,
                  msg.angular_velocity.y, msg.angular_velocity.z)
        if all(math.isfinite(v) for v in values):
            self.samples.append((time.monotonic(), tuple(float(v) for v in values)))

    def collect(self):
        self.active, self.samples = True, []
        deadline = time.monotonic() + float(self.p["message_timeout"])
        first = sample_start = None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            if self.samples and first is None:
                first = self.samples[0][0]
                sample_start = first + float(self.p["warmup_duration"])
                self.get_logger().info(f"Warm-up: {self.p['warmup_duration']:.1f} s")
            if first is None and now >= deadline:
                raise RuntimeError(f"No IMU messages on {self.p['input_topic']}")
            if sample_start and now >= sample_start + float(self.p["sample_duration"]):
                break
        self.active = False
        return [s for s in self.samples if s[0] >= sample_start]

    def run(self):
        print("="*60 + "\nLPMS-IG1 VEHICLE MOUNTING CALIBRATION\n" + "="*60)
        print("Estimate mounting roll/pitch. Static gravity CANNOT determine yaw.")
        print("Vehicle must be stationary on flat ground with IMU permanently installed.")
        input("Press ENTER when ready. ")
        samples = self.collect()
        count = len(samples)
        if count < int(self.p["minimum_samples"]):
            raise RuntimeError(f"Too few samples: {count}")
        channels = [[s[1][i] for s in samples] for i in range(6)]
        means = tuple(sum(c)/count for c in channels[:3])
        norms = [math.sqrt(x*x+y*y+z*z) for x,y,z in zip(*channels[:3])]
        gyro_norms = [math.sqrt(x*x+y*y+z*z) for x,y,z in zip(*channels[3:])]
        motion = sum(v > float(self.p["max_stationary_gyro"]) for v in gyro_norms)
        motion_ratio = motion/count
        mean_norm = sum(norms)/count
        if motion_ratio > float(self.p["max_motion_ratio"]):
            raise RuntimeError(f"vehicle moved: motion ratio {motion_ratio:.1%}")
        if abs(mean_norm-float(self.p["gravity_m_s2"])) > float(self.p["gravity_tolerance"]):
            raise RuntimeError(f"invalid gravity magnitude: {mean_norm:.5f} m/s^2")
        roll, pitch = estimate_mounting_rp(means)
        corrected = rotate_vector(rpy_matrix(roll, pitch, self.yaw), means)
        residual = math.hypot(corrected[0], corrected[1])
        if residual >= float(self.p["max_horizontal_residual"]):
            raise RuntimeError(f"horizontal residual too large: {residual:.5f} m/s^2")
        duration = max(samples[-1][0]-samples[0][0], 1e-9)
        self.report(count, duration, means, mean_norm, motion, motion_ratio,
                    roll, pitch, corrected, residual)
        self.save(roll, pitch)

    def report(self, count, duration, means, norm, motion, ratio, roll, pitch,
               corrected, residual):
        lines = ["="*60, "LPMS-IG1 VEHICLE MOUNTING RESULT", "="*60,
                 f"Samples: {count}; duration: {duration:.2f} s; rate: {count/duration:.1f} Hz",
                 f"Mean accel [m/s^2]: X={means[0]:+.6f} Y={means[1]:+.6f} Z={means[2]:+.6f}",
                 f"Magnitude: {norm:.6f} m/s^2; motion: {motion}/{count} ({ratio:.1%})",
                 f"base_link -> imu_link RPY [deg]: {math.degrees(roll):+.6f}, "
                 f"{math.degrees(pitch):+.6f}, {math.degrees(self.yaw):+.6f} (yaw user supplied)",
                 f"Corrected accel: X={corrected[0]:+.6f} Y={corrected[1]:+.6f} Z={corrected[2]:+.6f}",
                 f"Horizontal residual: {residual:.6f} m/s^2", "Calibration status: PASS", "="*60]
        for line in lines:
            self.get_logger().info(line)
        self.get_logger().warning("Accuracy is limited by vehicle floor/ground level.")
        self.get_logger().info("Yaw was NOT estimated from gravity.")

    def save(self, roll, pitch):
        q = quaternion_from_rpy(roll, pitch, self.yaw)
        text = (f"lpms_ig1_mounting:\n  parent_frame: {self.p['parent_frame']}\n"
                f"  child_frame: {self.p['child_frame']}\n  translation_m:\n"
                f"    x: {self.translation[0]:.12g}\n    y: {self.translation[1]:.12g}\n"
                f"    z: {self.translation[2]:.12g}\n  rotation_rpy_deg:\n"
                f"    roll: {math.degrees(roll):.12g}\n    pitch: {math.degrees(pitch):.12g}\n"
                f"    yaw: {math.degrees(self.yaw):.12g}\n  rotation_quaternion_xyzw:\n"
                f"    x: {q[0]:.12g}\n    y: {q[1]:.12g}\n    z: {q[2]:.12g}\n    w: {q[3]:.12g}\n")
        path = self.p["output_file"]
        path.parent.mkdir(parents=True, exist_ok=True)
        temp = path.with_name(path.name+".tmp")
        temp.write_text(text, encoding="utf-8")
        os.replace(temp, path)
        self.get_logger().info(f"Mounting configuration written to {path}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MountingCalibration(); node.run(); return 0
    except KeyboardInterrupt:
        if node: node.get_logger().info("Mounting calibration cancelled by user.")
    except (EOFError, OSError, RuntimeError, ValueError) as exc:
        if node: node.get_logger().error(f"Mounting calibration FAILED: {exc}")
        else: print(f"ERROR: {exc}")
    finally:
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
