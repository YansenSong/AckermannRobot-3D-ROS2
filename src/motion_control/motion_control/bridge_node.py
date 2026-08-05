#!/usr/bin/env python3
"""
UART Vehicle Bridge Node

Subscribes to /cmd_vel (geometry_msgs/Twist), converts to the STM32 26-byte
control protocol, and sends it as a UDP datagram to the motion control board.

/cmd_vel contract for this Ackermann vehicle:
  - linear.x: longitudinal speed (m/s)
  - angular.z: front-wheel steering angle (rad), not yaw rate

Protocol: Chapter 3 of 普通车运控项目通信协议说明_V1.0
  - 26-byte fixed frame, big-endian
  - Header "cmd__" (5 bytes)
  - Version, Mode, Flags, EnableMask
  - V_RAW (int32 BE): target speed m/s × 10000
  - EPS_RAW (int32 BE): steering angle in 0.1° units × 10000
  - SEB_MODE, SEB_VALUE, Light, Counter
  - Checksum: uint32 BE sum of bytes 0-21

NeuPAN already outputs the Ackermann steering angle, so this bridge must not
apply another kinematic conversion.
"""

import math
import socket
import struct
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist


class VehicleBridgeNode(Node):
    """Bridge /cmd_vel to the STM32 motion control board over UDP."""

    # Protocol constants
    HEADER = b"cmd__"  # bytes 0-4
    VERSION = 0x01     # byte 5
    MODE_AUTO = 0x01   # byte 6
    SPEED_SCALE = 10000       # v(m/s) → v_raw
    EPS_SCALE = 10000         # EPS raw (0.1° units) × 10000 → protocol field
    EPS_DEG_PER_RAW = 0.1     # degrees per EPS raw unit (matches uart_vehicle_bridge)
    FRAME_LENGTH = 26
    CHECKSUM_OFFSET = 22      # checksum covers bytes 0-21
    SHUTDOWN_STOP_FRAME_COUNT = 3
    SHUTDOWN_STOP_INTERVAL = 0.02

    def __init__(self):
        super().__init__('vehicle_bridge_node')

        # --- Declare parameters ---
        self.declare_parameter('udp_host', '192.168.1.100')
        self.declare_parameter('udp_port', 2000)
        self.declare_parameter('bind_device', '')
        self.declare_parameter('bind_interface', '')
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('max_reverse_speed', -0.5)
        self.declare_parameter('max_steer_deg', 30.0)
        self.declare_parameter('enable_mask', 7)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('command_timeout', 0.5)

        # --- Read parameters ---
        self._udp_host: str = self.get_parameter('udp_host').value
        self._udp_port: int = self.get_parameter('udp_port').value
        self._bind_device: str = self.get_parameter('bind_device').value
        self._bind_interface: str = self.get_parameter('bind_interface').value
        self._max_speed: float = self.get_parameter('max_speed').value
        self._max_reverse_speed: float = self.get_parameter(
            'max_reverse_speed').value
        self._max_steer_deg: float = self.get_parameter(
            'max_steer_deg').value
        self._enable_mask: int = self.get_parameter('enable_mask').value
        self._publish_rate: float = self.get_parameter('publish_rate').value
        self._command_timeout: float = self.get_parameter(
            'command_timeout').value

        # --- State ---
        self._counter: int = 0
        self._last_cmd_vel: Optional[Twist] = None
        self._last_cmd_time: Optional[Time] = None

        # --- UDP socket ---
        self._sock = self._create_udp_socket()
        self._target = (self._udp_host, self._udp_port)
        self.get_logger().info(
            f'UDP socket ready, target: {self._udp_host}:{self._udp_port}'
        )

        # --- Subscriber ---
        self._sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10
        )
        self.get_logger().info('Subscribed to /cmd_vel')

        # --- Timer: send frames at publish_rate Hz ---
        timer_period = 1.0 / self._publish_rate
        self._timer = self.create_timer(timer_period, self._timer_callback)
        self.get_logger().info(
            f'Publish timer started: {self._publish_rate:.1f} Hz '
            f'(every {timer_period*1000:.0f} ms)'
        )

        self.get_logger().info('Vehicle bridge node initialized')

    # ------------------------------------------------------------------
    # UDP socket
    # ------------------------------------------------------------------

    def _create_udp_socket(self) -> socket.socket:
        """Create and configure the UDP socket."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Allow address reuse (useful if restarting the node)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Bind to a specific network device (e.g., enp5s0) via SO_BINDTODEVICE.
        # This forces all packets through the named interface regardless of
        # the kernel routing table — critical for multi-NIC hosts with
        # overlapping subnets.
        if self._bind_device:
            try:
                # SO_BINDTODEVICE = 25 (Linux-specific)
                sock.setsockopt(socket.SOL_SOCKET, 25,
                                self._bind_device.encode('utf-8'))
                self.get_logger().info(
                    f'UDP socket bound to device {self._bind_device}'
                )
            except OSError as e:
                self.get_logger().error(
                    f'Failed to bind to device {self._bind_device}: {e}. '
                    f'Using default routing.'
                )

        # Bind to specific interface IP (source address) if requested
        if self._bind_interface:
            try:
                sock.bind((self._bind_interface, 0))
                self.get_logger().info(
                    f'UDP socket bound to address {self._bind_interface}'
                )
            except OSError as e:
                self.get_logger().error(
                    f'Failed to bind to {self._bind_interface}: {e}. '
                    f'Using default routing.'
                )

        return sock

    # ------------------------------------------------------------------
    # Subscriber callback
    # ------------------------------------------------------------------

    def _cmd_vel_callback(self, msg: Twist):
        """Store the latest cmd_vel message with timestamp."""
        self._last_cmd_vel = msg
        self._last_cmd_time = self.get_clock().now()

    # ------------------------------------------------------------------
    # Timer callback: main control loop
    # ------------------------------------------------------------------

    def _timer_callback(self):
        """Called at publish_rate Hz. Build frame and send via UDP."""
        now = self.get_clock().now()

        # Check for command timeout → safety stop
        if self._last_cmd_time is None:
            if self._counter == 0:
                self.get_logger().warn(
                    'No cmd_vel received yet, sending zero-velocity frame'
                )
            v = 0.0
            steer_rad = 0.0
        else:
            dt = (now - self._last_cmd_time).nanoseconds * 1e-9
            if dt > self._command_timeout:
                if self._counter % 20 == 0:  # throttle warning to ~1 Hz
                    self.get_logger().warn(
                        f'cmd_vel timeout '
                        f'({dt:.2f}s > {self._command_timeout}s), '
                        f'sending zero-velocity frame'
                    )
                v = 0.0
                steer_rad = 0.0
            else:
                v = self._last_cmd_vel.linear.x
                steer_rad = self._last_cmd_vel.angular.z

        # Reject malformed commands. Python min/max can turn NaN into a limit,
        # which would otherwise convert an invalid command into full actuation.
        v, steer_rad, command_is_valid = self._sanitize_command(v, steer_rad)
        if not command_is_valid:
            if self._counter % 20 == 0:
                self.get_logger().error(
                    'Non-finite cmd_vel received; sending stop command'
                )

        # angular.z is already the front-wheel steering angle in radians.
        eps_raw = self._compute_eps_raw(steer_rad)

        # Build 26-byte frame
        frame = self._build_frame(v, eps_raw)

        # Send via UDP
        try:
            self._sock.sendto(frame, self._target)
        except OSError as e:
            self.get_logger().error(f'UDP send failed: {e}')
            return

        # Log at DEBUG level
        hex_str = ' '.join(f'{b:02X}' for b in frame)
        self.get_logger().debug(
            f'Sent frame (cnt={self._counter:3d}): v={v:+.3f} m/s, '
            f'eps_raw={eps_raw:+d}, hex=[{hex_str}]'
        )

        # Increment counter (wraps at 255)
        self._counter = (self._counter + 1) % 256

    # ------------------------------------------------------------------
    # Steering protocol conversion
    # ------------------------------------------------------------------

    def _sanitize_command(self, speed: float, steer_rad: float):
        """Validate a command and clamp speed; invalid input means stop."""
        if not math.isfinite(speed) or not math.isfinite(steer_rad):
            return 0.0, 0.0, False
        speed = max(self._max_reverse_speed, min(self._max_speed, speed))
        return speed, steer_rad, True

    def _compute_eps_raw(self, steer_rad: float) -> int:
        """
        Convert a front-wheel steering angle in radians to protocol EPS raw.

        Protocol: EPS raw = steering angle in units of 0.1° (deg / 0.1);
        _build_frame then scales by EPS_SCALE (× 10000), so the wire field
        equals angle_deg × 100000. This matches the empirically verified
        uart_vehicle_bridge; the protocol doc's "degrees × 10000" alone does
        not match the STM32 firmware.
        """
        steer_deg = math.degrees(steer_rad)

        # Clamp to configured max steering
        steer_deg = max(
            -self._max_steer_deg,
            min(self._max_steer_deg, steer_deg),
        )

        return int(round(steer_deg / self.EPS_DEG_PER_RAW))

    # ------------------------------------------------------------------
    # Frame builder
    # ------------------------------------------------------------------

    def _build_frame(self, v: float, eps_raw: int) -> bytes:
        """
        Build the 26-byte UART protocol frame.

        Frame layout (all multi-byte integers big-endian):
            Bytes   Field       Description
            0-4     Header      "cmd__" (0x63 0x6D 0x64 0x5F 0x5F)
            5       Version     0x01
            6       Mode        0x01 (Auto)
            7       Flags       0x00
            8       EnableMask  bit0=RT49, bit1=EPS, bit2=SEB
            9-12    V_RAW       int32: v(m/s) × 10000
            13-16   EPS_RAW     int32: steering angle(°) / 0.1° × 10000
            17      SEB_MODE    0 (no brake control)
            18-19   SEB_VALUE   int16: 0
            20      Light       0
            21      Counter     0-255 rolling counter
            22-25   Checksum    uint32: sum of bytes 0-21
        """
        v_raw = int(round(v * self.SPEED_SCALE))
        eps_raw_scaled = int(round(eps_raw * self.EPS_SCALE))
        # Pack payload bytes (bytes 0-21)
        payload = struct.pack(
            '>5sBBBBi i B h B B',
            self.HEADER,                           # 0-4: header
            self.VERSION,                          # 5:   version
            self.MODE_AUTO,                        # 6:   mode
            0x00,                                  # 7:   flags
            self._enable_mask & 0x07,             # 8:   enable_mask
            v_raw,                                 # 9-12: V_RAW (int32 BE)
            eps_raw_scaled,                        # 13-16: EPS_RAW (int32 BE)
            0x00,                                  # 17: SEB_MODE
            0,                                # 18-19: SEB_VALUE (int16 BE)
            0x00,                                  # 20: Light
            self._counter & 0xFF,                 # 21: Counter
        )

        # Compute checksum: sum of bytes 0-21
        checksum = sum(payload) & 0xFFFFFFFF

        # Append checksum as uint32 BE
        frame = payload + struct.pack('>I', checksum)

        return frame

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def _send_shutdown_stop_frames(self):
        """Send repeated zero commands before closing the UDP socket."""
        for index in range(self.SHUTDOWN_STOP_FRAME_COUNT):
            frame = self._build_frame(0.0, 0)
            try:
                self._sock.sendto(frame, self._target)
            except OSError as exc:
                self.get_logger().error(
                    f'Failed to send shutdown stop frame: {exc}'
                )
                break
            self._counter = (self._counter + 1) % 256
            if index + 1 < self.SHUTDOWN_STOP_FRAME_COUNT:
                time.sleep(self.SHUTDOWN_STOP_INTERVAL)

    def destroy_node(self):
        """Clean up resources before shutdown."""
        if hasattr(self, '_sock') and self._sock:
            try:
                self._send_shutdown_stop_frames()
                self.get_logger().info('Shutdown stop frames sent')
            finally:
                self._sock.close()
                self._sock = None
                self.get_logger().info('UDP socket closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VehicleBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
