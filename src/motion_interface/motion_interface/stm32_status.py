#!/usr/bin/env python3
"""Receive, validate and decode the STM32 ``sta__`` status feedback.

The board pushes 56-byte ``sta__`` datagrams to the host at ~50 Hz; see
docs/上位机接口说明_STA56.md.

This node is deliberately passive: it declares no publisher and never sends a
control frame. That is what lets it perform the "receive status only, issue no
control" bench check as a single command.

The frame layout, validation and decoding live in ``motion_interface.sta56``,
which has no ROS dependency. This module is only the ROS plumbing around it:
binding the socket, draining it on a timer, and reporting. Anything that knows
what a byte means belongs in sta56, not here.
"""

import time
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node

from motion_interface.sta56 import (
    STATUS_FRAME_LENGTH,
    SUPERSEDED_FRAME_LENGTH,
    VALID_FLAG_NAMES,
    StatusFrame,
    bits_set,
    decode_status_frame,
    validate_status_frame,
)
from motion_interface.stm32_link import bind_udp_socket

# Read size, deliberately larger than a valid frame. A truncated read would
# make an oversized datagram look like a well-formed one, so the buffer must
# be big enough to see the true length and reject it.
STATUS_MAX_DATAGRAM = 2048


class Stm32StatusNode(Node):
    """Receive, validate, decode and report on the STM32 feedback stream."""

    REJECTION_REASONS = ('length', 'header', 'version', 'checksum')

    def __init__(self):
        super().__init__('stm32_status')

        self.declare_parameter('bind_device', 'enp5s0')
        self.declare_parameter('bind_interface', '192.168.5.11')
        self.declare_parameter('status_port', 5001)
        self.declare_parameter('poll_period', 0.05)
        self.declare_parameter('feedback_timeout', 0.2)
        self.declare_parameter('raw_dump_count', 3)
        self.declare_parameter('detail_period', 1.0)
        self.declare_parameter('summary_period', 5.0)
        self.declare_parameter('receive_buffer_bytes', 1 << 20)
        self.declare_parameter('max_drain_per_poll', 256)

        self._bind_device: str = self.get_parameter('bind_device').value
        self._bind_interface: str = self.get_parameter('bind_interface').value
        self._status_port: int = self.get_parameter('status_port').value
        self._poll_period: float = self.get_parameter('poll_period').value
        self._feedback_timeout: float = self.get_parameter(
            'feedback_timeout').value
        self._raw_dump_count: int = self.get_parameter('raw_dump_count').value
        self._detail_period: float = self.get_parameter('detail_period').value
        self._summary_period: float = self.get_parameter(
            'summary_period').value
        self._receive_buffer_bytes: int = self.get_parameter(
            'receive_buffer_bytes').value
        self._max_drain_per_poll: int = self.get_parameter(
            'max_drain_per_poll').value

        if self._poll_period <= 0.0:
            raise ValueError('poll_period must be greater than zero')
        if self._feedback_timeout <= 0.0:
            raise ValueError('feedback_timeout must be greater than zero')
        if self._detail_period <= 0.0:
            raise ValueError('detail_period must be greater than zero')
        if self._summary_period <= 0.0:
            raise ValueError('summary_period must be greater than zero')
        if not 0 < self._status_port < 65536:
            raise ValueError('status_port must be a valid UDP port')
        if self._raw_dump_count < 0:
            raise ValueError('raw_dump_count must be >= 0')
        if self._max_drain_per_poll <= 0:
            raise ValueError('max_drain_per_poll must be greater than zero')

        self._valid = 0
        self._rejected = {reason: 0 for reason in self.REJECTION_REASONS}
        self._recent_rx = deque()
        self._frames_dumped = 0
        self._activity_since_summary = False

        # The most recent decoded frame, re-reported on detail_period so the
        # decoded view keeps scrolling even when the log is otherwise quiet.
        self._detail_frame: Optional[StatusFrame] = None
        self._next_detail: Optional[float] = None

        # Feedback-loss tracking runs on the host's monotonic clock, as the
        # protocol note requires. Deliberately not the rclpy clock: this is an
        # observation about the host's own timeline and must not move if
        # use_sim_time is ever turned on.
        self._last_rx_monotonic: Optional[float] = None
        self._loss_started_at: Optional[float] = None

        self._sock = self._create_socket()
        self.get_logger().info(
            f'Listening for {STATUS_FRAME_LENGTH}-byte sta__ on '
            f'{self._bind_interface}:{self._status_port} '
            f'(device {self._bind_device})'
        )
        self.get_logger().info(
            f'Feedback-loss threshold: {self._feedback_timeout * 1000:.0f} ms '
            '(host-side observation only; silence does not prove the board '
            'stopped)'
        )

        self._next_summary = time.monotonic() + self._summary_period
        self._timer = self.create_timer(self._poll_period, self._poll)
        self.get_logger().info('STM32 status receiver initialized')

    def _create_socket(self):
        """Create and bind the receive socket.

        A failed bind is fatal here rather than a fallback: a receive-only node
        that silently is not bound to its port would just sit there reporting
        silence, which is indistinguishable from the board being off.
        """
        sock = bind_udp_socket(
            device=self._bind_device,
            address=self._bind_interface,
            port=self._status_port,
            for_receive=True,
            receive_buffer_bytes=self._receive_buffer_bytes,
            logger=self.get_logger(),
        )
        bound_port = sock.getsockname()[1]
        if bound_port != self._status_port:
            sock.close()
            raise RuntimeError(
                f'Could not bind UDP port {self._status_port} on '
                f'{self._bind_interface or "0.0.0.0"} (kernel gave port '
                f'{bound_port}). Another process may hold the port, or the '
                f'address may not exist on this host.'
            )
        return sock

    def _poll(self):
        """Drain the socket, then report on loss, decode and rate."""
        self._drain()
        now = time.monotonic()
        self._check_feedback_loss(now)
        if now >= self._next_summary:
            self._report_summary(now)
            self._next_summary = now + self._summary_period
        if self._next_detail is not None and now >= self._next_detail:
            self._report_detail()

    def _drain(self):
        """Read every datagram currently queued, without blocking."""
        drained = 0
        while drained < self._max_drain_per_poll:
            try:
                frame, source = self._sock.recvfrom(STATUS_MAX_DATAGRAM)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().error(f'UDP receive failed: {exc}')
                return
            drained += 1
            self._handle_frame(frame, source)

        # More is queued than one poll should consume. Say so and let the next
        # tick take the rest rather than spinning inside the callback.
        self.get_logger().warn(
            f'Drained {self._max_drain_per_poll} datagrams in one poll with '
            'more still queued; the board may be sending faster than the poll '
            'period can keep up with'
        )

    def _handle_frame(self, frame: bytes, source):
        now = time.monotonic()
        self._activity_since_summary = True

        reason = validate_status_frame(frame)
        if reason is not None:
            self._rejected[reason] += 1
            self._log_rejection(reason, frame, source)
            return

        self._valid += 1
        self._recent_rx.append(now)

        if self._frames_dumped < self._raw_dump_count:
            self._frames_dumped += 1
            self._dump_frame(frame, source)

        decoded = decode_status_frame(frame)
        self._detail_frame = decoded
        if self._next_detail is None:
            self._next_detail = now + self._detail_period

        if self._loss_started_at is not None:
            self.get_logger().info(
                f'Feedback resumed after '
                f'{now - self._loss_started_at:.3f} s of silence'
            )
            self._loss_started_at = None

        self._last_rx_monotonic = now

    def _check_feedback_loss(self, now: float):
        if self._last_rx_monotonic is None or self._loss_started_at is not None:
            return
        silence = now - self._last_rx_monotonic
        if silence < self._feedback_timeout:
            return
        self._loss_started_at = self._last_rx_monotonic
        self.get_logger().warn(
            f'No sta__ feedback for {silence:.3f} s '
            f'(threshold {self._feedback_timeout:.3f} s). This is a host-side '
            'observation threshold, not a board stop condition: receiving '
            'nothing does not prove the board stopped, and receiving '
            'something does not prove the actuators executed.'
        )

    def _log_rejection(self, reason: str, frame: bytes, source):
        count = self._rejected[reason]
        # The first few of each kind in full, then throttled: a mismatched
        # frame revision would otherwise print 50 lines a second forever.
        if count <= 3:
            preview = ' '.join(f'{byte:02X}' for byte in frame[:24])
            hint = ''
            if reason == 'length' and len(frame) == SUPERSEDED_FRAME_LENGTH:
                hint = (
                    f' -- a {SUPERSEDED_FRAME_LENGTH}-byte frame is the '
                    'superseded revision; this node follows the 56-byte layout '
                    'of 上位机接口说明_STA56.md, and Version is 1 in both so '
                    'length is the only discriminator'
                )
            self.get_logger().warn(
                f'Rejected sta__ frame ({reason}) from '
                f'{source[0]}:{source[1]}; len={len(frame)}, '
                f'first 24 bytes=[{preview}]{hint}'
            )
        elif count % 100 == 0:
            self.get_logger().warn(
                f'Rejected {count} sta__ frames so far, all as ({reason})'
            )

    def _dump_frame(self, frame: bytes, source):
        lines = [
            f'sta__ raw frame {self._frames_dumped}/{self._raw_dump_count} '
            f'from {source[0]}:{source[1]} ({len(frame)} bytes)'
        ]
        for offset in range(0, len(frame), 16):
            chunk = frame[offset:offset + 16]
            hex_part = ' '.join(f'{byte:02X}' for byte in chunk)
            lines.append(f'  {offset:04d}  {hex_part}')
        self.get_logger().info('\n'.join(lines))

    def _report_detail(self):
        """The decoded view, repeated on detail_period so it keeps scrolling."""
        frame = self._detail_frame
        if frame is None:
            return
        self._next_detail = time.monotonic() + self._detail_period
        self.get_logger().info(frame.summary())

        # Only surface the flag lists when something is actually set. A healthy
        # board sets the convention disclosures, so an unconditional dump would
        # train the reader to ignore this line.
        faults = frame.subsystem_fault_names
        if faults:
            self.get_logger().info(f'  FaultFlags: {", ".join(faults)}')
        missing = [
            f'{name} ({frame.why_invalid(bit)})'
            for bit, name in VALID_FLAG_NAMES.items()
            if not frame.has(bit)
        ]
        if missing:
            self.get_logger().info(
                f'  ValidFlags clear: {", ".join(missing)}'
            )

    def _report_summary(self, now: float):
        if not self._activity_since_summary:
            # Silence is already reported by _check_feedback_loss; a periodic
            # "still nothing" line would only bury it.
            return
        self._activity_since_summary = False

        while self._recent_rx and now - self._recent_rx[0] > self._summary_period:
            self._recent_rx.popleft()

        rate = 0.0
        if len(self._recent_rx) >= 2:
            span = self._recent_rx[-1] - self._recent_rx[0]
            if span > 0.0:
                rate = (len(self._recent_rx) - 1) / span

        rejected_total = sum(self._rejected.values())
        parts = [f'valid={self._valid} ({rate:.1f} Hz)']
        if rejected_total:
            detail = ', '.join(
                f'{reason}={count}'
                for reason, count in self._rejected.items()
                if count
            )
            parts.append(f'rejected={rejected_total} ({detail})')
        else:
            parts.append('rejected=0')
        if self._last_rx_monotonic is not None:
            parts.append(f'silence={(now - self._last_rx_monotonic) * 1000:.0f} ms')

        self.get_logger().info(f'sta__: {" | ".join(parts)}')

    def destroy_node(self):
        """Close the receive socket and report the run totals."""
        if self._sock is not None:
            rejected_total = sum(self._rejected.values())
            self.get_logger().info(
                f'sta__ totals: valid={self._valid}, '
                f'rejected={rejected_total} '
                f'({", ".join(f"{r}={c}" for r, c in self._rejected.items())})'
            )
            self._sock.close()
            self._sock = None
            self.get_logger().info('UDP receive socket closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Stm32StatusNode()
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
