import math
import struct

from motion_control.bridge_node import VehicleBridgeNode


class FakeSocket:
    def __init__(self):
        self.datagrams = []

    def sendto(self, frame, target):
        self.datagrams.append((frame, target))


def make_bridge(enable_mask=3, counter=0):
    bridge = object.__new__(VehicleBridgeNode)
    bridge._max_steer_deg = 30.0
    bridge._enable_mask = enable_mask
    bridge._counter = counter
    return bridge


def unpack_control_fields(frame):
    return struct.unpack('>ii', frame[9:17])


def test_steering_angle_uses_protocol_degree_scale():
    bridge = make_bridge()

    # EPS raw is in 0.1° units; _build_frame scales ×10000 → field = deg × 100000.
    # 30° → 300 (0.1° units); 45° clamps to 30° → 300.
    assert bridge._compute_eps_raw(math.radians(30.0)) == 300
    assert bridge._compute_eps_raw(math.radians(-30.0)) == -300
    assert bridge._compute_eps_raw(math.radians(45.0)) == 300


def test_frame_matches_reference_steering_scale():
    """30° must produce EPS field 3,000,000 (0x2DC6C0), matching the
    empirically-verified uart_vehicle_bridge — not the doc's 300,000."""
    bridge = make_bridge(enable_mask=3, counter=1)
    eps_raw = bridge._compute_eps_raw(math.radians(30.0))

    frame = bridge._build_frame(0.2, eps_raw)

    expected = bytes.fromhex(
        '63 6D 64 5F 5F 01 01 00 03 00 00 07 D0 00 2D C6 C0 '
        '00 00 00 00 01 00 00 04 82'
    )
    assert frame == expected
    assert len(frame) == VehicleBridgeNode.FRAME_LENGTH
    assert unpack_control_fields(frame) == (2_000, 3_000_000)


def test_non_finite_command_is_replaced_with_stop():
    bridge = make_bridge()
    bridge._max_speed = 2.0
    bridge._max_reverse_speed = -0.5

    assert bridge._sanitize_command(float('nan'), 0.0) == (0.0, 0.0, False)
    assert bridge._sanitize_command(0.5, float('nan')) == (0.0, 0.0, False)
    assert bridge._sanitize_command(float('inf'), 0.0) == (0.0, 0.0, False)
    assert bridge._sanitize_command(3.0, 0.1) == (2.0, 0.1, True)
    assert bridge._sanitize_command(-1.0, -0.1) == (-0.5, -0.1, True)


def test_shutdown_sends_repeated_zero_frames():
    bridge = make_bridge(enable_mask=7, counter=254)
    bridge._sock = FakeSocket()
    bridge._target = ('192.168.1.50', 5000)
    bridge.SHUTDOWN_STOP_FRAME_COUNT = 3
    bridge.SHUTDOWN_STOP_INTERVAL = 0.0

    bridge._send_shutdown_stop_frames()

    assert len(bridge._sock.datagrams) == 3
    assert bridge._counter == 1
    for frame, target in bridge._sock.datagrams:
        assert target == bridge._target
        assert unpack_control_fields(frame) == (0, 0)
