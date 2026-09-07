#!/usr/bin/env python3
"""
Arrow key teleop for Ackermann robot with D/R gear.

  ↑ : accelerate (in current gear direction)
  ↓ : decelerate (toward zero)
  ← : steer left
  → : steer right
  B : toggle gear  D (forward) / R (reverse)
  Space : emergency stop (speed → 0)
  Q / Ctrl-C : quit

Publishes geometry_msgs/TwistStamped to /ackermann_steering_controller/reference.
"""

import os
import sys
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


# Arrow key escape sequences
UP    = b'\x1b[A'
DOWN  = b'\x1b[B'
RIGHT = b'\x1b[C'
LEFT  = b'\x1b[D'
SPACE = b' '
B_KEY_LOWER = b'b'
B_KEY_UPPER = b'B'
Q_KEY_LOWER = b'q'
Q_KEY_UPPER = b'Q'


class ArrowKeyTeleop(Node):
    def __init__(self):
        super().__init__('arrow_key_teleop')

        self.pub = self.create_publisher(TwistStamped, '/ackermann_steering_controller/reference', 10)

        # Parameters
        self.declare_parameter('max_linear', 2.0)
        self.declare_parameter('max_angular', 0.6)
        self.declare_parameter('linear_step', 0.2)
        self.declare_parameter('angular_step', 0.05)

        self.max_linear   = self.get_parameter('max_linear').value
        self.max_angular  = self.get_parameter('max_angular').value
        self.linear_step  = self.get_parameter('linear_step').value
        self.angular_step = self.get_parameter('angular_step').value

        # D = forward (positive speed), R = reverse (negative speed)
        self.gear = 'D'
        self.linear = 0.0    # always ≥ 0 (speed magnitude)
        self.angular = 0.0
        self.running = True

        self.print_status()

        # Publish continuously at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_cmd)
        # Keyboard reading thread
        self.key_thread = threading.Thread(target=self.read_keys, daemon=True)
        self.key_thread.start()

        self.get_logger().info('ArrowKeyTeleop started.')

    # ----------------------------------------------------------------
    def _cmd_linear(self):
        """Signed velocity: positive in D, negative in R."""
        return self.linear if self.gear == 'D' else -self.linear

    # ----------------------------------------------------------------
    def print_status(self):
        gear_display = f'[{self.gear}]'
        print(f'\r  GEAR={gear_display}  speed={self._cmd_linear():+.1f} m/s  '
              f'angle={self.angular:+.2f} rad   ', end='', flush=True)

    # ----------------------------------------------------------------
    def publish_cmd(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = self._cmd_linear()
        msg.twist.angular.z = self.angular
        self.pub.publish(msg)

    # ----------------------------------------------------------------
    def read_keys(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            buf = b''
            while self.running:
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    ch = os.read(fd, 1)
                    if ch == b'\x1b':
                        buf = b'\x1b'
                    elif buf == b'\x1b' and ch == b'[':
                        buf = b'\x1b['
                    elif buf == b'\x1b[':
                        seq = buf + ch
                        buf = b''
                        self.handle_key(seq)
                    else:
                        buf = b''
                        self.handle_key(ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # ----------------------------------------------------------------
    def handle_key(self, key):
        if key == UP:
            self.linear = min(self.linear + self.linear_step, self.max_linear)
            self.print_status()
        elif key == DOWN:
            self.linear = max(self.linear - self.linear_step, 0.0)
            self.print_status()
        elif key == LEFT:
            self.angular = min(self.angular + self.angular_step, self.max_angular)
            self.print_status()
        elif key == RIGHT:
            self.angular = max(self.angular - self.angular_step, -self.max_angular)
            self.print_status()
        elif key == SPACE:
            self.linear = 0.0
            self.angular = 0.0
            print('\n  ■  EMERGENCY STOP — speed=0')
        elif key in (B_KEY_LOWER, B_KEY_UPPER):
            self.gear = 'R' if self.gear == 'D' else 'D'
            self.linear = 0.0   # reset speed on gear change
            self.angular = 0.0
            print(f'\n  ⚙  Toggle to {"D (forward)" if self.gear == "D" else "R (reverse)"}')
        elif key in (Q_KEY_LOWER, Q_KEY_UPPER, b'\x03'):
            self.linear = 0.0
            self.angular = 0.0
            self.publish_cmd()
            print('\n  Quit.')
            self.running = False
            rclpy.shutdown()


# ====================================================================
def main():
    print("""
╔══════════════════════════════════════════╗
║   Arrow Key Teleop for Ackermann Robot   ║
╠══════════════════════════════════════════╣
║   ↑  : accelerate                        ║
║   ↓  : decelerate                        ║
║   ←  : steer left                        ║
║   →  : steer right                       ║
║   B  : toggle D (forward) / R (reverse)  ║
║   Space : emergency stop                 ║
║   Q  : quit                              ║
╠══════════════════════════════════════════╣
║   Default D gear (forward).              ║
║   Press B to switch to R (reverse).      ║
║   Speed resets on gear change.           ║
╚══════════════════════════════════════════╝
""")
    rclpy.init(args=sys.argv)
    node = ArrowKeyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
