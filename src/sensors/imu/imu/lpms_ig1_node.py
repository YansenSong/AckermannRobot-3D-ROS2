#!/usr/bin/env python3
import math
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


G0 = 9.80665

CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x000007FF
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000

CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)


def quaternion_multiply_wxyz(q1, q2):
    """Hamilton product, both quaternions are (w, x, y, z)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )


def quaternion_normalize_wxyz(q):
    n = math.sqrt(sum(v*v for v in q))
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(v / n for v in q)


class LpmsIg1Ros2(Node):
    """
    LPMS-IG1 CANopen 16-bit -> ROS 2 sensor_msgs driver.

    Default CAN mapping used:
      0x180 + node_id: AccCal X/Y/Z, GyroII AlignCal X
      0x280 + node_id: GyroII AlignCal Y/Z, MagCal X/Y
      0x380 + node_id: MagCal Z, Euler Roll/Pitch/Yaw
      0x480 + node_id: Quaternion W/X/Y/Z

    Publishes:
      imu/data_raw : sensor_msgs/msg/Imu
      imu/data     : sensor_msgs/msg/Imu
      imu/mag      : sensor_msgs/msg/MagneticField

    ROS conversions:
      acceleration: g -> m/s^2, and optionally sign inverted so a stationary
                    IMU with +Z up reports +9.80665 m/s^2 on Z (REP-145)
      gyro:         deg/s -> rad/s
      magnetometer: microtesla -> tesla
      quaternion:   optionally LPMS NWU world reference -> ROS ENU world reference

    NOTE:
      This driver keeps measurements expressed in the LPMS sensor/body axes.
      Use a static TF from base_link -> imu_link (or LPMS Object Reset) to describe
      how the IMU is mounted on the robot.
    """

    def __init__(self):
        super().__init__("lpms_ig1_node")

        self.declare_parameter("interface", "can0")
        self.declare_parameter("node_id", 5)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("poll_period_sec", 0.001)
        self.declare_parameter("auto_reconnect", True)
        self.declare_parameter("reconnect_interval_sec", 2.0)
        self.declare_parameter("data_timeout_sec", 2.0)
        self.declare_parameter("status_period_sec", 0.5)

        # LPMS calibrated accelerometer in the observed/default convention gives
        # about -1 g on +Z when resting Z-up; REP-145 expects +g.
        self.declare_parameter("invert_accel_for_ros", True)

        # LPMS acc+gyr+mag global frame: X north, Y west, Z up (NWU).
        # ROS REP-103 navigation convention: X east, Y north, Z up (ENU).
        self.declare_parameter("convert_nwu_to_enu", True)

        # Unknown covariance -> all zeros is valid ROS convention.
        # If you have measured/calibrated standard deviations, set them here.
        self.declare_parameter("orientation_stddev", 0.0)          # rad
        self.declare_parameter("angular_velocity_stddev", 0.0)     # rad/s
        self.declare_parameter("linear_acceleration_stddev", 0.0)  # m/s^2
        self.declare_parameter("magnetic_field_stddev", 0.0)       # tesla

        # User-measured static gyro biases, applied only to imu/data. The raw
        # topic intentionally retains the converted but uncorrected readings.
        self.declare_parameter("gyro_bias_x", 0.0)  # rad/s
        self.declare_parameter("gyro_bias_y", 0.0)  # rad/s
        self.declare_parameter("gyro_bias_z", 0.0)  # rad/s

        # Per-axis accelerometer correction in ROS SI units. Applied only to
        # imu/data after sign and unit conversion.
        self.declare_parameter("accel_offset_x", 0.0)  # m/s^2
        self.declare_parameter("accel_offset_y", 0.0)  # m/s^2
        self.declare_parameter("accel_offset_z", 0.0)  # m/s^2
        self.declare_parameter("accel_gain_x", 1.0)
        self.declare_parameter("accel_gain_y", 1.0)
        self.declare_parameter("accel_gain_z", 1.0)

        self.interface = str(self.get_parameter("interface").value)
        self.node_id = int(self.get_parameter("node_id").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.poll_period = float(self.get_parameter("poll_period_sec").value)
        self.auto_reconnect = bool(self.get_parameter("auto_reconnect").value)
        self.reconnect_interval = float(
            self.get_parameter("reconnect_interval_sec").value
        )
        self.data_timeout = float(self.get_parameter("data_timeout_sec").value)
        self.status_period = float(self.get_parameter("status_period_sec").value)
        self.invert_accel = bool(self.get_parameter("invert_accel_for_ros").value)
        self.nwu_to_enu = bool(self.get_parameter("convert_nwu_to_enu").value)
        self.gyro_bias = (
            float(self.get_parameter("gyro_bias_x").value),
            float(self.get_parameter("gyro_bias_y").value),
            float(self.get_parameter("gyro_bias_z").value),
        )
        self.accel_offset = (
            float(self.get_parameter("accel_offset_x").value),
            float(self.get_parameter("accel_offset_y").value),
            float(self.get_parameter("accel_offset_z").value),
        )
        self.accel_gain = (
            float(self.get_parameter("accel_gain_x").value),
            float(self.get_parameter("accel_gain_y").value),
            float(self.get_parameter("accel_gain_z").value),
        )

        if not (1 <= self.node_id <= 127):
            raise ValueError("node_id must be in the CANopen range 1..127")
        for name, value in (
            ("poll_period_sec", self.poll_period),
            ("reconnect_interval_sec", self.reconnect_interval),
            ("data_timeout_sec", self.data_timeout),
            ("status_period_sec", self.status_period),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
        if not all(math.isfinite(value) for value in self.accel_offset):
            raise ValueError("accelerometer offsets must be finite")
        if not all(math.isfinite(value) and value > 0.0 for value in self.accel_gain):
            raise ValueError("accelerometer gains must be finite and > 0")

        self.ids = {
            0x180 + self.node_id: 1,
            0x280 + self.node_id: 2,
            0x380 + self.node_id: 3,
            0x480 + self.node_id: 4,
        }

        self.pub_raw = self.create_publisher(
            Imu, "imu/data_raw", qos_profile_sensor_data
        )
        self.pub_imu = self.create_publisher(
            Imu, "imu/data", qos_profile_sensor_data
        )
        self.pub_mag = self.create_publisher(
            MagneticField, "imu/mag", qos_profile_sensor_data
        )
        self.pub_connected = self.create_publisher(Bool, "imu/connected", 10)
        self.reconnect_service = self.create_service(
            SetBool, "imu/enable_auto_reconnect", self.set_auto_reconnect
        )

        self.sock = None
        self.socket_connected_at = None
        self.last_complete_sample_at = None
        self.next_reconnect_at = 0.0
        self.last_reported_connected = None
        self.values = {}
        self.mask = 0

        self.try_connect(force=True)
        self.timer = self.create_timer(self.poll_period, self.poll_can)
        self.status_timer = self.create_timer(
            self.status_period, self.publish_connection_status
        )

        ids_text = ", ".join(f"0x{x:03X}" for x in self.ids)
        self.get_logger().info(
            f"LPMS-IG1 on {self.interface}, node_id={self.node_id}, "
            f"frame_id={self.frame_id}, CAN IDs=[{ids_text}]"
        )
        self.get_logger().info(
            f"ROS conversion: invert_accel={self.invert_accel}, "
            f"NWU->ENU orientation={self.nwu_to_enu}"
        )
        self.get_logger().info(
            "Gyro bias [rad/s]: "
            f"x={self.gyro_bias[0]:+.9f}, y={self.gyro_bias[1]:+.9f}, "
            f"z={self.gyro_bias[2]:+.9f}"
        )
        self.get_logger().info(
            "Accel calibration: offset [m/s^2]="
            f"({self.accel_offset[0]:+.6f}, {self.accel_offset[1]:+.6f}, "
            f"{self.accel_offset[2]:+.6f}), gain="
            f"({self.accel_gain[0]:.6f}, {self.accel_gain[1]:.6f}, "
            f"{self.accel_gain[2]:.6f})"
        )

    def try_connect(self, force=False):
        if self.sock is not None:
            return True
        now = time.monotonic()
        if not force and (
            not self.auto_reconnect or now < self.next_reconnect_at
        ):
            return False

        new_sock = None
        try:
            new_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            new_sock.setblocking(False)
            new_sock.bind((self.interface,))
        except OSError as exc:
            if new_sock is not None:
                try:
                    new_sock.close()
                except OSError:
                    pass
            self.next_reconnect_at = now + self.reconnect_interval
            self.get_logger().error(
                f"Cannot open CAN interface {self.interface}: {exc}"
            )
            return False

        self.sock = new_sock
        self.socket_connected_at = time.monotonic()
        self.last_complete_sample_at = None
        self.values = {}
        self.mask = 0
        self.get_logger().info(f"Bound CAN interface {self.interface}")
        return True

    def close_socket(self):
        sock = self.sock
        self.sock = None
        self.socket_connected_at = None
        self.last_complete_sample_at = None
        self.values = {}
        self.mask = 0
        if sock is not None:
            try:
                sock.close()
            except OSError as exc:
                self.get_logger().warning(f"CAN socket close failed: {exc}")

    def disconnect(self, reason):
        self.close_socket()
        self.next_reconnect_at = time.monotonic() + self.reconnect_interval
        self.get_logger().warning(f"CAN disconnected: {reason}")

    def set_auto_reconnect(self, request, response):
        enabled = bool(request.data)
        result = self.set_parameters(
            [Parameter("auto_reconnect", value=enabled)]
        )[0]
        if not result.successful:
            response.success = False
            response.message = result.reason
            return response

        self.auto_reconnect = enabled
        if self.auto_reconnect and self.sock is None:
            self.next_reconnect_at = 0.0
        response.success = True
        response.message = (
            "CAN auto reconnect enabled" if self.auto_reconnect
            else "CAN auto reconnect disabled"
        )
        self.get_logger().info(response.message)
        return response

    def publish_connection_status(self):
        now = time.monotonic()
        last_activity = (
            self.last_complete_sample_at
            if self.last_complete_sample_at is not None
            else self.socket_connected_at
        )
        if (
            self.sock is not None
            and last_activity is not None
            and now - last_activity > self.data_timeout
            and self.auto_reconnect
        ):
            self.disconnect(f"no complete IMU sample for {self.data_timeout:.1f}s")

        connected = (
            self.sock is not None
            and self.last_complete_sample_at is not None
            and now - self.last_complete_sample_at <= self.data_timeout
        )
        status = Bool()
        status.data = connected
        self.pub_connected.publish(status)
        if connected != self.last_reported_connected:
            if connected:
                self.get_logger().info("IMU data connected")
            else:
                self.get_logger().warning("IMU data unavailable")
            self.last_reported_connected = connected

    @staticmethod
    def unpack_i16x4(data):
        if len(data) != 8:
            raise ValueError("LPMS 16-bit CAN frame must contain 8 data bytes")
        return struct.unpack("<hhhh", data)

    def decode_frame(self, can_id, data):
        group = self.ids.get(can_id)
        if group is None or len(data) != 8:
            return

        a, b, c, d = self.unpack_i16x4(data)

        # A new TPDO1 marks a new sample set. If the prior set was incomplete,
        # discard it instead of mixing frames from different samples.
        if group == 1:
            self.mask = 0
            self.values = {}
            self.values["ax_g"] = a / 1000.0
            self.values["ay_g"] = b / 1000.0
            self.values["az_g"] = c / 1000.0
            self.values["gx_dps"] = d / 10.0
            self.mask |= 0b0001

        elif group == 2:
            self.values["gy_dps"] = a / 10.0
            self.values["gz_dps"] = b / 10.0
            self.values["mx_uT"] = c / 100.0
            self.values["my_uT"] = d / 100.0
            self.mask |= 0b0010

        elif group == 3:
            self.values["mz_uT"] = a / 100.0
            # Euler is decoded for completeness/debugging but the ROS Imu
            # orientation field is populated from the quaternion.
            self.values["roll_deg"] = b / 100.0
            self.values["pitch_deg"] = c / 100.0
            self.values["yaw_deg"] = d / 100.0
            self.mask |= 0b0100

        elif group == 4:
            self.values["qw"] = a / 10000.0
            self.values["qx"] = b / 10000.0
            self.values["qy"] = c / 10000.0
            self.values["qz"] = d / 10000.0
            self.mask |= 0b1000

            if self.mask == 0b1111:
                self.publish_sample(self.values)
                self.last_complete_sample_at = time.monotonic()

            self.mask = 0
            self.values = {}

    @staticmethod
    def diag_covariance(stddev):
        if stddev <= 0.0:
            return [0.0] * 9
        var = stddev * stddev
        return [
            var, 0.0, 0.0,
            0.0, var, 0.0,
            0.0, 0.0, var,
        ]

    def publish_sample(self, v):
        stamp = self.get_clock().now().to_msg()

        # --- acceleration ---
        # Convert LPMS g to SI. For ROS REP-145, an upward-pointing Z axis at
        # rest should read +g, while this LPMS mapping/setting was observed as -g.
        accel_sign = -1.0 if self.invert_accel else 1.0
        ax_raw = accel_sign * v["ax_g"] * G0
        ay_raw = accel_sign * v["ay_g"] * G0
        az_raw = accel_sign * v["az_g"] * G0
        ax = (ax_raw - self.accel_offset[0]) * self.accel_gain[0]
        ay = (ay_raw - self.accel_offset[1]) * self.accel_gain[1]
        az = (az_raw - self.accel_offset[2]) * self.accel_gain[2]

        # --- gyro ---
        deg_to_rad = math.pi / 180.0
        gx_raw = v["gx_dps"] * deg_to_rad
        gy_raw = v["gy_dps"] * deg_to_rad
        gz_raw = v["gz_dps"] * deg_to_rad
        gx = gx_raw - self.gyro_bias[0]
        gy = gy_raw - self.gyro_bias[1]
        gz = gz_raw - self.gyro_bias[2]

        # --- orientation ---
        q = quaternion_normalize_wxyz(
            (v["qw"], v["qx"], v["qy"], v["qz"])
        )

        if self.nwu_to_enu:
            # Coordinates: NWU -> ENU is +90 deg about +Z.
            # If q maps sensor/body coordinates into LPMS NWU world,
            # q_enu = q(ENU<-NWU) * q_nwu.
            s = math.sqrt(0.5)
            q_enu_from_nwu = (s, 0.0, 0.0, s)  # +90 deg about Z
            q = quaternion_normalize_wxyz(
                quaternion_multiply_wxyz(q_enu_from_nwu, q)
            )

        qw, qx, qy, qz = q

        # --- full fused IMU ---
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.frame_id

        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw

        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        imu.orientation_covariance = self.diag_covariance(
            float(self.get_parameter("orientation_stddev").value)
        )
        imu.angular_velocity_covariance = self.diag_covariance(
            float(self.get_parameter("angular_velocity_stddev").value)
        )
        imu.linear_acceleration_covariance = self.diag_covariance(
            float(self.get_parameter("linear_acceleration_stddev").value)
        )

        self.pub_imu.publish(imu)

        # --- raw IMU topic: same accel/gyro, explicitly no orientation ---
        raw = Imu()
        raw.header.stamp = stamp
        raw.header.frame_id = self.frame_id
        raw.orientation.w = 1.0
        raw.orientation_covariance[0] = -1.0
        raw.angular_velocity.x = gx_raw
        raw.angular_velocity.y = gy_raw
        raw.angular_velocity.z = gz_raw
        raw.linear_acceleration.x = ax_raw
        raw.linear_acceleration.y = ay_raw
        raw.linear_acceleration.z = az_raw
        raw.angular_velocity_covariance = imu.angular_velocity_covariance
        raw.linear_acceleration_covariance = imu.linear_acceleration_covariance

        self.pub_raw.publish(raw)

        # --- magnetometer ---
        mag = MagneticField()
        mag.header.stamp = stamp
        mag.header.frame_id = self.frame_id
        mag.magnetic_field.x = v["mx_uT"] * 1.0e-6
        mag.magnetic_field.y = v["my_uT"] * 1.0e-6
        mag.magnetic_field.z = v["mz_uT"] * 1.0e-6
        mag.magnetic_field_covariance = self.diag_covariance(
            float(self.get_parameter("magnetic_field_stddev").value)
        )

        self.pub_mag.publish(mag)

    def poll_can(self):
        if self.sock is None:
            self.try_connect()
            return
        while rclpy.ok():
            try:
                frame = self.sock.recv(CAN_FRAME_SIZE)
            except BlockingIOError:
                return
            except OSError as exc:
                self.disconnect(f"receive error: {exc}")
                return

            if len(frame) != CAN_FRAME_SIZE:
                continue

            can_id_raw, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)

            if can_id_raw & (CAN_RTR_FLAG | CAN_ERR_FLAG):
                continue

            can_id = can_id_raw & CAN_EFF_MASK
            if can_id <= CAN_SFF_MASK:
                can_id &= CAN_SFF_MASK

            self.decode_frame(can_id, data[:dlc])

    def destroy_node(self):
        try:
            self.close_socket()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LpmsIg1Ros2()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
