#!/usr/bin/env python3
"""The 56-byte ``sta__`` status frame: layout, validation and decoding.

Pure protocol logic, deliberately free of any ROS dependency so it can be
imported by the rclpy node, by standalone tools, and by tests without a ROS
environment. Socket and checksum plumbing lives in ``stm32_link``.

Field layout, units, validity bits and enum values all come from
docs/上位机接口说明_STA56.md.

Two things that document is emphatic about and this decoder therefore
preserves:

  * A byte value of 0 does NOT mean "invalid". Validity is carried by
    ValidFlags (per field) and FaultFlags (per subsystem); a valid 0 m/s with
    a fresh RT49 is a real measurement of standing still. Reporting 0 for an
    invalid field is the single easiest way to mislead an operator.
  * The frame is 56 bytes, NOT the earlier 48-byte revision. Both carry
    Version=1, so **length is the only thing that distinguishes them**.
    Misreading a 48-byte frame as this layout would misparse every field past
    offset 5 without any error surfacing.
"""

import struct
from dataclasses import dataclass
from typing import Optional

from motion_interface.stm32_link import checksum32

STATUS_FRAME_LENGTH = 56
STATUS_HEADER = b'sta__'
STATUS_HEADER_LENGTH = 5
STATUS_VERSION_OFFSET = 5
STATUS_VERSION = 0x01
# The checksum covers every byte before it, so it always sits at length-4.
STATUS_CHECKSUM_OFFSET = STATUS_FRAME_LENGTH - 4

# The superseded revision. Recognised only so rejections can say which
# revision arrived instead of just "wrong length".
SUPERSEDED_FRAME_LENGTH = 48

CONTROL_SOURCE_NAMES = {
    0: 'safe/no-fresh-command',
    1: 'udp',
    2: 'rc',
    3: 'usart1',
    4: 'internal/debug',
}

BRAKE_MODE_NAMES = {
    0: 'unknown/fault',
    1: 'pressure',
    2: 'stroke',
}

SEB_FAULT_LEVEL_NAMES = {0: 'none', 1: 'minor', 2: 'normal', 3: 'severe'}

EPS_STATE_NAMES = {
    0: 'undefined',
    1: 'controllable',
    2: 'temporarily-uncontrollable',
    3: 'severe-fault',
}

# Field widths, needed for correct wrap-around arithmetic. Counter and
# LastCmdCounter are u8 (the document is explicit that Counter wraps 0..255),
# while TimestampMs is u32 -- mixing the widths is a silent-wrong-answer bug.
COUNTER_BITS = 8
TIMESTAMP_MS_BITS = 32

VALID_FLAG_NAMES = {
    0: 'drive-encoder',
    1: 'actual-speed',
    2: 'actual-steering-angle',
    3: 'steering-raw',
    4: 'brake-actual',
    5: 'rt49-status',
    6: 'eps-status',
    7: 'seb-status',
}

# FaultFlags. Only 0..10 come from the requirements document; 11..20 are the
# firmware's frozen diagnostic extensions. Several are mode/口径 disclosures
# rather than faults -- bits 11, 19 and 20 in particular are set on a healthy
# board, so they must not be read as a fault summary.
FAULT_FLAG_NAMES = {
    0: 'rt49-status-stale',
    1: 'eps-status-stale',
    2: 'seb-status-stale',
    3: 'rt49-fault-code',
    4: 'eps-fault',
    5: 'seb-fault',
    6: 'estop-requested-in-command',
    7: 'command-invalid-or-stale',
    8: 'rc-takeover',
    9: 'drive-encoder-unavailable',
    10: 'equivalent-front-wheel-angle-uncalibrated',
    11: 'speed-scale-unconfirmed',
    12: 'rc-takeover-lost',
    13: 'rt49-enable-unknown',
    14: 'eps-enable-unknown',
    15: 'seb-enable-unknown',
    16: 'eps-fault-tables-incomplete',
    17: 'brake-actual-invalid-for-mode',
    18: 'no-command-accepted-yet',
    19: 'legacy-speed-coefficient',
    20: 'direct-eps-angle-convention',
}

# Fault bits that disclose an uncalibrated convention rather than a fault.
# The document labels 19 and 20 that way explicitly ("是口径说明，不是...");
# 11 is grouped with them because it is a statement about the conversion
# parameter being unconfirmed, and it too is set on a healthy board. All three
# are filtered out of the INVALID explanations so a real cause is not buried.
CONVENTION_FLAGS = (11, 19, 20)

# ValidFlags bit <-> the human name for the measurement it validates.
SPEED_VALID_BIT = 1
STEERING_VALID_BIT = 2
STEERING_RAW_VALID_BIT = 3
BRAKE_VALID_BIT = 4


def validate_status_frame(frame: bytes) -> Optional[str]:
    """Return None if ``frame`` is a well-formed ``sta__`` frame, else why not.

    Unknown lengths and versions are rejected and recorded rather than
    accepted, and a datagram being received says nothing about whether its
    contents are valid -- so this is a gate, not a formality.
    """
    if len(frame) != STATUS_FRAME_LENGTH:
        return 'length'
    if frame[:STATUS_HEADER_LENGTH] != STATUS_HEADER:
        return 'header'
    if frame[STATUS_VERSION_OFFSET] != STATUS_VERSION:
        return 'version'

    expected = int.from_bytes(frame[STATUS_CHECKSUM_OFFSET:], 'big')
    if checksum32(frame[:STATUS_CHECKSUM_OFFSET]) != expected:
        return 'checksum'
    return None


def bits_set(value: int, names: dict) -> list:
    """Names of the set bits in ``value``, in ascending bit order."""
    return [name for bit, name in sorted(names.items()) if value & (1 << bit)]


def _signed16(raw: bytes) -> int:
    return struct.unpack('>h', raw)[0]


@dataclass(frozen=True)
class StatusFrame:
    """One decoded sta__ frame. See docs/上位机接口说明_STA56.md."""

    valid_flags: int
    control_source: int
    enable_state: int
    counter: int
    timestamp_ms: int
    drive_encoder_count: int
    actual_speed_mps: float
    actual_steering_deg: float
    steering_raw: int
    brake_mode: int
    brake_actual: Optional[float]
    gear: int
    rt49_status: int
    eps_status: int
    seb_status: int
    fault_flags: int
    last_cmd_counter: int

    def has(self, bit: int) -> bool:
        return bool(self.valid_flags & (1 << bit))

    @property
    def speed_valid(self) -> bool:
        return self.has(SPEED_VALID_BIT)

    @property
    def steering_valid(self) -> bool:
        return self.has(STEERING_VALID_BIT)

    @property
    def steering_raw_valid(self) -> bool:
        return self.has(STEERING_RAW_VALID_BIT)

    @property
    def brake_actual_text(self) -> str:
        """BrakeActual in the unit its mode implies, or why it has no unit.

        The wire value is scaled: MPa x100 in pressure mode, mm x10 in stroke
        mode. The divisor therefore depends on the mode, which is why the raw
        field is kept unscaled on the dataclass.
        """
        if not self.has(BRAKE_VALID_BIT) or self.brake_actual is None:
            return 'INVALID'
        if self.brake_mode == 1:
            return f'{self.brake_actual / 100.0:.2f} MPa'
        if self.brake_mode == 2:
            return f'{self.brake_actual / 10.0:.1f} mm'
        return 'INVALID (mode unknown)'

    @property
    def gear_text(self) -> str:
        return {1: 'forward', -1: 'reverse'}.get(self.gear, 'stopped/unknown')

    @property
    def rt49_fault_code(self) -> int:
        return self.rt49_status & 0xFF

    @property
    def eps_mode(self) -> int:
        return self.eps_status & 0x1

    @property
    def eps_state(self) -> int:
        return (self.eps_status >> 1) & 0x3

    @property
    def seb_fault_level(self) -> int:
        return (self.seb_status >> 6) & 0x3

    @property
    def fault_names(self) -> list:
        return bits_set(self.fault_flags, FAULT_FLAG_NAMES)

    @property
    def subsystem_fault_names(self) -> list:
        """Fault names excluding the uncalibrated-convention disclosures."""
        return [
            name
            for name in self.fault_names
            if name not in {
                FAULT_FLAG_NAMES[bit] for bit in CONVENTION_FLAGS
            }
        ]

    def why_invalid(self, bit: int) -> str:
        """Best explanation for a cleared ValidFlags bit, from FaultFlags.

        A cleared valid bit on its own says "do not trust this number" but not
        why, and the why is almost always in FaultFlags. Convention
        disclosures are filtered out here: they are set on a healthy board and
        would otherwise bury the actual cause.
        """
        causes = {
            SPEED_VALID_BIT: ('rt49-status-stale',),
            STEERING_VALID_BIT: ('eps-status-stale', 'eps-fault'),
            STEERING_RAW_VALID_BIT: (
                'eps-status-stale', 'eps-fault-tables-incomplete',
            ),
            BRAKE_VALID_BIT: (
                'seb-status-stale', 'seb-fault', 'brake-actual-invalid-for-mode',
            ),
        }.get(bit, ())
        matched = [c for c in causes if c in self.fault_names]
        return ', '.join(matched) if matched else 'reason not reported'

    def summary(self) -> str:
        """One-line decoded view, with INVALID where a valid bit is clear."""
        speed = (
            f'{self.actual_speed_mps:+.4f} m/s'
            if self.speed_valid else 'INVALID'
        )
        # Deliberately not labelled "front-wheel angle": the firmware's
        # direct-EPS-angle convention is flagged uncalibrated (FaultFlags
        # bit 20), so this is the firmware's number, not a road-wheel angle.
        steering = (
            f'{self.actual_steering_deg:+.4f} deg'
            if self.steering_valid else 'INVALID'
        )
        steering_raw = (
            f'{self.steering_raw:+d} deg-count'
            if self.steering_raw_valid else 'INVALID'
        )
        return (
            f'cnt={self.counter:3d} t={self.timestamp_ms}ms '
            f'src={CONTROL_SOURCE_NAMES.get(self.control_source, self.control_source)} '
            f'en={self.enable_state:#04x} '
            f'speed={speed} steer={steering} raw={steering_raw} '
            f'brake={BRAKE_MODE_NAMES.get(self.brake_mode, self.brake_mode)}'
            f'/{self.brake_actual_text} gear={self.gear_text} '
            f'rt49={self.rt49_status:#06x}(fault={self.rt49_fault_code}) '
            f'eps={self.eps_status:#06x} seb={self.seb_status:#06x} '
            f'lastcmd={self.last_cmd_counter} '
            # "vflags", not "valid": the statistics line already uses "valid"
            # for the received-frame count, and two different meanings for one
            # word in the same log is how a reader misdiagnoses a healthy board.
            f'vflags={self.valid_flags:#04x} fault={self.fault_flags:#010x}'
        )


def decode_status_frame(frame: bytes) -> StatusFrame:
    """Decode a frame that has already passed :func:`validate_status_frame`."""
    return StatusFrame(
        valid_flags=frame[6],
        control_source=frame[7],
        enable_state=frame[8],
        counter=frame[9],
        timestamp_ms=int.from_bytes(frame[10:14], 'big'),
        drive_encoder_count=int.from_bytes(frame[14:22], 'big', signed=True),
        actual_speed_mps=int.from_bytes(
            frame[22:26], 'big', signed=True
        ) / 10000.0,
        actual_steering_deg=int.from_bytes(
            frame[26:30], 'big', signed=True
        ) / 10000.0,
        steering_raw=int.from_bytes(frame[30:34], 'big', signed=True),
        brake_mode=frame[34],
        # The divisor for this depends on brake_mode, so it is kept as the
        # raw signed integer and converted in brake_actual_text.
        brake_actual=float(_signed16(frame[35:37])),
        gear=struct.unpack('>b', frame[37:38])[0],
        rt49_status=int.from_bytes(frame[38:40], 'big'),
        eps_status=int.from_bytes(frame[40:42], 'big'),
        seb_status=int.from_bytes(frame[42:44], 'big'),
        fault_flags=int.from_bytes(frame[44:48], 'big'),
        last_cmd_counter=frame[48],
    )
