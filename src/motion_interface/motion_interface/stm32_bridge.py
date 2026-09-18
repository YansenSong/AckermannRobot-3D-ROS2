#!/usr/bin/env python3
"""Bridge the canonical Ackermann ROS command to the STM32 UDP protocol.

Subscribes to the canonical Ackermann command (geometry_msgs/Twist), converts
it to the STM32 26-byte control protocol, and sends it as UDP datagrams to the
motion-control board.

Canonical command contract (default topic: /ackermann_cmd):
  - linear.x: longitudinal speed (m/s)
  - angular.z: front-wheel steering angle (rad), not body yaw rate

Protocol: section 3 of docs/上位机接口说明_STA56.md (2026-09-17)
  - 26-byte fixed frame, big-endian
  - Header "cmd__" (5 bytes)
  - Version, Mode, Flags, EnableMask
  - V_RAW (int32 BE): target speed m/s × 10000
  - EPS_RAW (int32 BE): steering angle in 0.1° units × 10000
  - SEB_MODE, SEB_VALUE, Light, Counter
  - Checksum: uint32 BE sum of bytes 0-21

The status feedback the board sends back on UDP 5001 is handled by the
separate stm32_status node in this package.

The bridge intentionally performs no bicycle-model conversion: angular.z is
already the front-wheel steering angle. Planning/control layers must preserve
that canonical command contract before the command reaches this hardware edge.
"""

import math
import socket
import struct
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time

from motion_interface.stm32_link import bind_udp_socket, checksum32


class Stm32VehicleBridgeNode(Node):
    """Bridge the canonical Ackermann command to the STM32 board over UDP."""

    HEADER = b"cmd__"
    VERSION = 0x01
    MODE_DISABLE = 0x00
    MODE_AUTO = 0x01
    SPEED_SCALE = 10000
    EPS_SCALE = 10000
    EPS_DEG_PER_RAW = 0.1
    FRAME_LENGTH = 26
    CHECKSUM_OFFSET = 22
    SHUTDOWN_STOP_FRAME_COUNT = 3
    SHUTDOWN_STOP_INTERVAL = 0.02

    def __init__(self):
        super().__init__('stm32_vehicle_bridge')

        self.declare_parameter('command_topic', '/ackermann_cmd')
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
        self.declare_parameter('arm_on_first_command', True)

        self._command_topic: str = self.get_parameter('command_topic').value
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
        self._arm_on_first_command: bool = self.get_parameter(
            'arm_on_first_command').value

        if self._publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')
        if self._command_timeout <= 0.0:
            raise ValueError('command_timeout must be greater than zero')

        # Until the first live command is adopted, frame mode/enable mask are
        # held at "disable / nothing enabled" rather than "auto / all enabled".
        # See _timer_callback for why arming is tied to command arrival.
        self._armed: bool = not self._arm_on_first_command

        self._counter: int = 0
        self._last_command: Optional[Twist] = None
        self._last_command_time: Optional[Time] = None

        self._sock = self._create_udp_socket()
        self._target = (self._udp_host, self._udp_port)
        self.get_logger().info(
            f'UDP socket ready, target: {self._udp_host}:{self._udp_port}'
        )

        self._sub = self.create_subscription(
            Twist, self._command_topic, self._command_callback, 10
        )
        self.get_logger().info(
            f'Subscribed to {self._command_topic} '
            '(linear.x=speed, angular.z=steering angle)'
        )

        timer_period = 1.0 / self._publish_rate
        self._timer = self.create_timer(timer_period, self._timer_callback)
        self.get_logger().info(
            f'Publish timer started: {self._publish_rate:.1f} Hz '
            f'(every {timer_period*1000:.0f} ms)'
        )
        if not self._armed:
            self.get_logger().info(
                'Holding actuators disarmed (mode=disable, enable_mask=0) '
                f'until the first valid command arrives on '
                f'{self._command_topic}'
            )
        self.get_logger().info('STM32 vehicle bridge initialized')

    def _create_udp_socket(self) -> socket.socket:
        """Create and configure the send-side UDP socket.

        Port 0 asks the kernel for an ephemeral source port: the protocol note
        (section 2) only constrains the destination, so any local source port
        is accepted and pinning a specific one would only risk a conflict.
        """
        return bind_udp_socket(
            device=self._bind_device,
            address=self._bind_interface,
            port=0,
            logger=self.get_logger(),
        )

    def _command_callback(self, msg: Twist):
        """Store the latest canonical Ackermann command with its timestamp."""
        self._last_command = msg
        self._last_command_time = self.get_clock().now()

    def _timer_callback(self):
        """Build and transmit one protocol frame at the configured rate."""
        now = self.get_clock().now()
        used_live_command = False

        if self._last_command_time is None:
            if self._counter == 0:
                self.get_logger().warn(
                    f'No command received on {self._command_topic}; '
                    'sending zero-velocity frame'
                )
            speed = 0.0
            steer_rad = 0.0
        else:
            age = (now - self._last_command_time).nanoseconds * 1e-9
            if age > self._command_timeout:
                if self._counter % 20 == 0:
                    self.get_logger().warn(
                        f'Command timeout ({age:.2f}s > '
                        f'{self._command_timeout:.2f}s); sending stop frame'
                    )
                speed = 0.0
                steer_rad = 0.0
            else:
                speed = self._last_command.linear.x
                steer_rad = self._last_command.angular.z
                used_live_command = True

        speed, steer_rad, command_is_valid = self._sanitize_command(
            speed, steer_rad
        )
        if not command_is_valid and self._counter % 20 == 0:
            self.get_logger().error(
                'Non-finite Ackermann command received; sending stop frame'
            )

        # Arm on the first command that is both fresh and usable, not on the
        # first non-zero speed. A live command on the topic means the upstream
        # control chain is up: the command gate publishes nothing at all until
        # its planner has sent something, so before that there is no window in
        # which arming could be triggered by a stale snapshot.
        #
        # Arming is one-way. A later timeout must not disarm, because that
        # would flap the mode (and a disarmed mode makes the board recentre the
        # steering) every time the planner briefly stalls.
        if used_live_command and command_is_valid and not self._armed:
            self._armed = True
            self.get_logger().info(
                f'First live command on {self._command_topic}; arming '
                f'(mode={self.MODE_AUTO:#04x}, '
                f'enable_mask={self._enable_mask:#04x})'
            )

        if self._armed:
            frame_mode, frame_enable_mask = self.MODE_AUTO, self._enable_mask
        else:
            frame_mode, frame_enable_mask = self.MODE_DISABLE, 0

        eps_raw = self._compute_eps_raw(steer_rad)
        frame = self._build_frame(
            speed, eps_raw, frame_mode, frame_enable_mask
        )

        try:
            self._sock.sendto(frame, self._target)
        except OSError as exc:
            self.get_logger().error(f'UDP send failed: {exc}')
            return

        hex_str = ' '.join(f'{byte:02X}' for byte in frame)
        self.get_logger().debug(
            f'Sent frame (cnt={self._counter:3d}): '
            f'v={speed:+.3f} m/s, steer={steer_rad:+.3f} rad, '
            f'eps_raw={eps_raw:+d}, hex=[{hex_str}]'
        )

        self._counter = (self._counter + 1) % 256

    def _sanitize_command(self, speed: float, steer_rad: float):
        """Validate a command and clamp speed; invalid input means stop."""
        if not math.isfinite(speed) or not math.isfinite(steer_rad):
            return 0.0, 0.0, False
        speed = max(self._max_reverse_speed, min(self._max_speed, speed))
        return speed, steer_rad, True

    def _compute_eps_raw(self, steer_rad: float) -> int:
        """Convert front-wheel steering angle in radians to protocol EPS raw."""
        steer_deg = math.degrees(steer_rad)
        steer_deg = max(
            -self._max_steer_deg,
            min(self._max_steer_deg, steer_deg),
        )
        return int(round(steer_deg / self.EPS_DEG_PER_RAW))

    def _build_frame(
        self,
        speed: float,
        eps_raw: int,
        mode: Optional[int] = None,
        enable_mask: Optional[int] = None,
    ) -> bytes:
        """Build the fixed 26-byte big-endian STM32 control frame.

        ``mode`` and ``enable_mask`` are encoder inputs rather than read from
        ``self._armed`` so the arming policy stays in exactly one place (the
        timer) and this stays a pure function of its arguments. Omitting them
        keeps the historical encoded values, which is what the shutdown frames
        and the reference-vector test both want.
        """
        speed_raw = int(round(speed * self.SPEED_SCALE))
        eps_raw_scaled = int(round(eps_raw * self.EPS_SCALE))
        frame_mode = self.MODE_AUTO if mode is None else mode
        frame_enable_mask = (
            self._enable_mask if enable_mask is None else enable_mask
        )

        payload = struct.pack(
            '>5sBBBBi i B h B B',
            self.HEADER,
            self.VERSION,
            frame_mode,
            0x00,
            frame_enable_mask & 0x07,
            speed_raw,
            eps_raw_scaled,
            0x00,
            0,
            0x00,
            self._counter & 0xFF,
        )
        return payload + struct.pack('>I', checksum32(payload))

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
    node = Stm32VehicleBridgeNode()
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
