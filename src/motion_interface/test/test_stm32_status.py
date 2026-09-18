import struct
import zlib
from collections import deque

from motion_interface.sta56 import (
    STATUS_CHECKSUM_OFFSET,
    STATUS_FRAME_LENGTH,
    STATUS_HEADER,
    STATUS_VERSION,
    SUPERSEDED_FRAME_LENGTH,
    decode_status_frame,
    validate_status_frame,
)
from motion_interface.stm32_link import checksum32, unsigned_delta
from motion_interface.stm32_status import Stm32StatusNode

# A frame captured from the real board on 2026-09-17, idle (no cmd__ sent yet),
# EPS not communicating. Used as the anchor vector: it exercises the real byte
# values, the exact checksum, and a fault set the firmware actually emits.
HARDWARE_FRAME = bytes.fromhex(
    '73 74 61 5F 5F 01 B2 00 00 79 00 06 C8 2E 00 00 '
    '00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 '
    '00 00 02 FF FD 00 09 00 00 00 00 A1 00 1D 66 A2 '
    '00 00 00 00 00 00 07 FB'
)

# 52 bytes before the checksum: header (5) + version (1) + the payload.
BODY_LENGTH = STATUS_CHECKSUM_OFFSET


def build_status_frame(**fields):
    """Build a well-formed 56-byte sta__ frame with a correct checksum."""
    body = bytearray(BODY_LENGTH)
    body[0:5] = STATUS_HEADER
    body[5] = fields.get('version', STATUS_VERSION)
    body[6] = fields.get('valid_flags', 0)
    body[7] = fields.get('control_source', 0)
    body[8] = fields.get('enable_state', 0)
    body[9] = fields.get('counter', 0)
    body[10:14] = int(fields.get('timestamp_ms', 0)).to_bytes(4, 'big')
    body[14:22] = int(fields.get('drive_encoder_count', 0)).to_bytes(
        8, 'big', signed=True
    )
    body[22:26] = int(fields.get('actual_speed_raw', 0)).to_bytes(
        4, 'big', signed=True
    )
    body[26:30] = int(fields.get('actual_steering_raw', 0)).to_bytes(
        4, 'big', signed=True
    )
    body[30:34] = int(fields.get('steering_raw', 0)).to_bytes(
        4, 'big', signed=True
    )
    body[34] = fields.get('brake_mode', 0)
    body[35:37] = struct.pack('>h', fields.get('brake_actual', 0))
    body[37:38] = struct.pack('>b', fields.get('gear', 0))
    body[38:40] = int(fields.get('rt49_status', 0)).to_bytes(2, 'big')
    body[40:42] = int(fields.get('eps_status', 0)).to_bytes(2, 'big')
    body[42:44] = int(fields.get('seb_status', 0)).to_bytes(2, 'big')
    body[44:48] = int(fields.get('fault_flags', 0)).to_bytes(4, 'big')
    body[48] = fields.get('last_cmd_counter', 0)
    assert len(body) == BODY_LENGTH
    return bytes(body) + struct.pack('>I', checksum32(bytes(body)))


class FakeLogger:
    def __init__(self):
        self.records = []

    def info(self, msg):
        self.records.append(('info', msg))

    def warn(self, msg):
        self.records.append(('warn', msg))

    def error(self, msg):
        self.records.append(('error', msg))

    def messages(self, level=None):
        return [
            msg for lvl, msg in self.records if level is None or lvl == level
        ]


def make_status_node(raw_dump_count=3, feedback_timeout=0.2):
    """Build a node without running Node.__init__, so no ROS graph is needed."""
    node = object.__new__(Stm32StatusNode)
    node._valid = 0
    node._rejected = {r: 0 for r in Stm32StatusNode.REJECTION_REASONS}
    node._recent_rx = deque()
    node._frames_dumped = 0
    node._raw_dump_count = raw_dump_count
    node._activity_since_summary = False
    node._detail_frame = None
    node._next_detail = None
    node._detail_period = 1.0
    node._last_rx_monotonic = None
    node._loss_started_at = None
    node._feedback_timeout = feedback_timeout
    node._summary_period = 5.0
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    return node


# --- frame gating -----------------------------------------------------------

def test_hardware_frame_is_accepted():
    assert len(HARDWARE_FRAME) == STATUS_FRAME_LENGTH
    assert validate_status_frame(HARDWARE_FRAME) is None


def test_superseded_48_byte_frame_is_rejected_as_length():
    """The 48-byte revision also used Version=1, so length is the only tell.

    Accepting it would mean misparsing every field past offset 5, so the
    rejection is the point of this test, not a side effect.
    """
    old_frame = HARDWARE_FRAME[:SUPERSEDED_FRAME_LENGTH]

    assert validate_status_frame(old_frame) == 'length'


def test_each_rejection_reason_is_attributed_separately():
    assert validate_status_frame(HARDWARE_FRAME[:-1]) == 'length'
    assert validate_status_frame(HARDWARE_FRAME + b'\x00') == 'length'
    assert validate_status_frame(b'') == 'length'

    bad_header = bytearray(HARDWARE_FRAME)
    bad_header[0] = ord('X')
    assert validate_status_frame(bytes(bad_header)) == 'header'

    # Version is checked before the checksum, so this reports 'version' even
    # though the mutation also invalidates the checksum.
    bad_version = bytearray(HARDWARE_FRAME)
    bad_version[5] = 2
    assert validate_status_frame(bytes(bad_version)) == 'version'

    bad_checksum = bytearray(HARDWARE_FRAME)
    bad_checksum[STATUS_CHECKSUM_OFFSET] ^= 0xFF
    assert validate_status_frame(bytes(bad_checksum)) == 'checksum'


def test_checksum_is_a_byte_sum_not_a_crc_or_word_sum():
    body = HARDWARE_FRAME[:STATUS_CHECKSUM_OFFSET]
    stored = int.from_bytes(HARDWARE_FRAME[STATUS_CHECKSUM_OFFSET:], 'big')

    assert stored == sum(body) == checksum32(body)
    assert stored != zlib.crc32(body)

    word_sum = sum(
        int.from_bytes(body[i:i + 4], 'big')
        for i in range(0, len(body) - (len(body) % 4), 4)
    )
    assert stored != word_sum


# --- decoding ---------------------------------------------------------------

def test_hardware_frame_decodes_to_expected_fields():
    f = decode_status_frame(HARDWARE_FRAME)

    assert f.valid_flags == 0xB2
    assert f.control_source == 0
    assert f.counter == 0x79
    assert f.timestamp_ms == 0x0006C82E
    assert f.drive_encoder_count == 0
    assert f.actual_speed_mps == 0.0
    assert f.brake_mode == 2
    assert f.brake_actual == -3.0
    assert f.gear == 0
    assert f.rt49_status == 0x0900
    assert f.eps_status == 0
    assert f.seb_status == 0x00A1
    assert f.fault_flags == 0x001D66A2
    assert f.last_cmd_counter == 0


def test_hardware_frame_validity_bits():
    f = decode_status_frame(HARDWARE_FRAME)

    # RT49 and SEB were fresh; EPS was not communicating at all.
    assert f.has(1)    # actual speed
    assert f.has(4)    # brake actual
    assert f.has(5)    # rt49 status
    assert f.has(7)    # seb status
    assert not f.has(6)  # eps status -- the board was not hearing CAN1
    assert not f.has(2)  # steering angle
    assert not f.has(3)  # steering raw
    assert not f.has(0)  # drive encoder -- never available


def test_hardware_frame_fault_flags():
    f = decode_status_frame(HARDWARE_FRAME)

    assert f.fault_flags & (1 << 1)    # eps status stale
    assert f.fault_flags & (1 << 5)    # seb fault level non-zero
    assert f.fault_flags & (1 << 7)    # command invalid or stale
    assert f.fault_flags & (1 << 18)   # no command accepted yet
    assert f.fault_flags & (1 << 19)   # legacy speed coefficient
    assert f.fault_flags & (1 << 20)   # direct eps angle convention
    # The requirements-derived fault bits. RT49's code was 0 here, so bit 3
    # must stay clear -- this is the cross-check that bit 8/11 in RT49Status
    # are status bits and not part of the fault code.
    assert not f.fault_flags & (1 << 3)
    assert f.rt49_fault_code == 0
    assert f.rt49_status & (1 << 11)   # zero RPM


def test_zero_speed_with_fresh_rt49_is_valid_not_missing():
    """A valid 0 is a measurement; the doc warns against treating 0 as invalid."""
    f = decode_status_frame(build_status_frame(valid_flags=0x02))

    assert f.actual_speed_mps == 0.0
    assert f.has(1)
    assert 'speed=+0.0000 m/s' in f.summary()


def test_speed_and_steering_scaling():
    f = decode_status_frame(
        build_status_frame(
            valid_flags=0x0E,
            actual_speed_raw=2000,
            actual_steering_raw=-50000,
            steering_raw=-50,
        )
    )

    assert f.actual_speed_mps == 0.2
    assert f.actual_steering_deg == -5.0
    # SteeringRaw carries the EPS's raw count, not degrees: at the confirmed
    # 0.1 deg/count it takes 50 counts to make -5 deg (-5.0 x 10 == -50).
    assert f.steering_raw == -50


def test_steering_scale_is_one_tenth_degree_per_count():
    """实车确认：原始计数 300 对应 30°，即 0.1deg/count，换算已在 STM32 内完成。

    The handover note is explicit that the host must NOT divide by ten again:
    the angle field already carries (count / 10) degrees x10000, so the host's
    only job is /10000. Getting this wrong is a silent 10x error in either
    direction -- 300 instead of 30, or 3 instead of 30 -- and it would look
    like a plausible steering angle either way, so the anchor the firmware
    quotes is pinned here rather than left to the reader.
    """
    f = decode_status_frame(
        build_status_frame(
            valid_flags=0x0C,
            actual_steering_raw=300_000,
            steering_raw=300,
        )
    )

    assert f.actual_steering_deg == 30.0
    assert f.steering_raw == 300
    assert f.steering_valid and f.steering_raw_valid
    # Both fields describe the same physical angle, so once the raw count is
    # converted they must agree. If the firmware ever applies the scale to
    # only one of them, this is the assertion that catches it.
    assert f.steering_raw / 10.0 == f.actual_steering_deg

    # The reference the field test compares against: left full lock.
    left_full = decode_status_frame(
        build_status_frame(
            valid_flags=0x0C, actual_steering_raw=274_000, steering_raw=274
        )
    )
    assert left_full.actual_steering_deg == 27.4


def test_negative_values_are_twos_complement():
    f = decode_status_frame(
        build_status_frame(
            actual_speed_raw=-2000,
            actual_steering_raw=-100000,
            steering_raw=-100,
            gear=-1,
        )
    )

    assert f.actual_speed_mps == -0.2
    assert f.actual_steering_deg == -10.0
    assert f.steering_raw == -100
    assert f.gear == -1
    assert f.gear_text == 'reverse'


def test_brake_actual_unit_follows_mode():
    pressure = decode_status_frame(
        build_status_frame(valid_flags=0x10, brake_mode=1, brake_actual=123)
    )
    stroke = decode_status_frame(
        build_status_frame(valid_flags=0x10, brake_mode=2, brake_actual=123)
    )

    assert pressure.brake_actual_text == '1.23 MPa'
    assert stroke.brake_actual_text == '12.3 mm'


def test_brake_actual_is_invalid_without_its_valid_bit():
    f = decode_status_frame(build_status_frame(valid_flags=0x00, brake_mode=1))

    assert f.brake_actual_text == 'INVALID'


def test_why_invalid_names_the_subsystem_cause():
    # EPS was not communicating on the captured frame, so the cleared steering
    # valid bits have to say so rather than leaving the reader to guess.
    f = decode_status_frame(HARDWARE_FRAME)

    assert 'eps-status-stale' in f.why_invalid(2)
    assert 'eps-status-stale' in f.why_invalid(3)
    # RT49 was fresh, so a valid speed must not be blamed on a stale RT49.
    assert f.speed_valid
    assert 'rt49-status-stale' not in f.why_invalid(1)


def test_convention_disclosures_are_not_reported_as_faults():
    """Bits 19/20 are set on a healthy board; surfacing them buries real faults."""
    f = decode_status_frame(HARDWARE_FRAME)

    assert 'legacy-speed-coefficient' in f.fault_names
    assert 'direct-eps-angle-convention' in f.fault_names
    assert 'legacy-speed-coefficient' not in f.subsystem_fault_names
    assert 'direct-eps-angle-convention' not in f.subsystem_fault_names
    # A real fault must survive the filter.
    assert 'eps-status-stale' in f.subsystem_fault_names


# --- node behaviour ---------------------------------------------------------

def test_valid_frames_are_counted_and_invalid_ones_are_not():
    node = make_status_node()

    for _ in range(3):
        node._handle_frame(HARDWARE_FRAME, ('192.168.5.50', 5001))
    assert node._valid == 3
    assert sum(node._rejected.values()) == 0

    node._handle_frame(HARDWARE_FRAME[:48], ('192.168.5.50', 5001))
    assert node._valid == 3
    assert node._rejected['length'] == 1


def test_rejecting_a_48_byte_frame_explains_the_revision_change():
    node = make_status_node()

    node._handle_frame(HARDWARE_FRAME[:48], ('192.168.5.50', 5001))

    warns = node._logger.messages('warn')
    assert any('48-byte frame is the superseded revision' in m for m in warns)


def test_only_the_first_n_frames_are_dumped_as_hex():
    node = make_status_node(raw_dump_count=2)

    for _ in range(4):
        node._handle_frame(HARDWARE_FRAME, ('192.168.5.50', 5001))

    dumps = [msg for msg in node._logger.messages() if 'raw frame' in msg]
    assert len(dumps) == 2
    assert '73 74 61 5F 5F 01 B2' in dumps[0]


def test_last_valid_frame_is_decoded_for_the_detail_view():
    node = make_status_node()

    node._handle_frame(HARDWARE_FRAME, ('192.168.5.50', 5001))
    assert node._detail_frame is not None
    assert node._detail_frame.counter == 0x79

    # A rejected frame must not overwrite the last good decode.
    node._handle_frame(b'short', ('192.168.5.50', 5001))
    assert node._detail_frame.counter == 0x79


def test_feedback_loss_and_resume_are_reported():
    node = make_status_node(feedback_timeout=0.2)

    node._handle_frame(HARDWARE_FRAME, ('192.168.5.50', 5001))
    last_rx = node._last_rx_monotonic

    node._check_feedback_loss(last_rx + 0.1)
    assert node._loss_started_at is None

    node._check_feedback_loss(last_rx + 0.5)
    assert node._loss_started_at == last_rx
    assert any('No sta__ feedback' in m for m in node._logger.messages('warn'))

    node._logger.records.clear()
    node._check_feedback_loss(last_rx + 1.0)
    assert node._logger.messages('warn') == []

    node._handle_frame(HARDWARE_FRAME, ('192.168.5.50', 5001))
    assert node._loss_started_at is None
    assert any('resumed' in m for m in node._logger.messages('info'))


# --- counter arithmetic -----------------------------------------------------

def test_counter_delta_handles_uint32_wrap():
    """The note is explicit: mask with 0xFFFFFFFF, never compare signed."""
    assert unsigned_delta(2, 0xFFFFFFFF) == 3
    assert unsigned_delta(100, 90) == 10
    assert 2 - 0xFFFFFFFF < 0
    assert unsigned_delta(2, 0xFFFFFFFF) > 0


def test_timestamp_ms_delta_handles_its_own_wrap():
    # TimestampMs wraps every ~49.7 days and is a different field from Counter,
    # so it needs its own check rather than being assumed to wrap alongside.
    assert unsigned_delta(1000, 0xFFFFFFFF - 999) == 2000
    assert unsigned_delta(0, 0xFFFFFFFF) == 1
