#!/usr/bin/env python3
"""Shared plumbing for the STM32 UDP link.

Both directions of the link need the same two Linux-specific socket options
(pin the socket to a network device, bind it to a local address) and the same
checksum algorithm. Keeping them here means the ``SO_BINDTODEVICE`` option
number and the checksum definition exist once rather than once per direction.

The two directions differ only in the port they bind:

  send     ``bind_udp_socket(device, address, 0)``     ephemeral source port.
           The protocol note (section 2) allows any local source port.

  receive  ``bind_udp_socket(device, address, 5001)``  the port the STM32 is
           told to send its ``sta__`` feedback to (section 1).
"""

import socket
from typing import Optional

# Linux-only socket option. Python's socket module has no constant for it, so
# it is written out here once instead of inline at each call site.
SO_BINDTODEVICE = 25


def checksum32(payload: bytes) -> int:
    """Return the protocol checksum for ``payload``.

    The protocol note is explicit (section 2 for ``cmd__``, section 3 for
    ``sta__``) that this is a plain byte-wise sum of everything before the
    checksum field, NOT a CRC and NOT a sum over 32-bit words. Both frame
    types use it, so both directions must call this rather than reimplement
    the loop.
    """
    return sum(payload) & 0xFFFFFFFF


def unsigned_delta(new: int, previous: int, width_bits: int = 32) -> int:
    """Difference between two unsigned protocol counters, across wrap-around.

    The protocol note spells out why this must not be a signed comparison: a
    signed subtraction reports a huge negative number at every wrap, so a
    naive "is this newer than that" test concludes the value is stale forever
    after the wrap.

    ``width_bits`` must match the field's actual width, and the counters on
    this link are not all the same size: the sta__ ``TimestampMs`` is uint32
    (wrapping every ~49.7 days), while ``Counter`` and ``LastCmdCounter`` are
    uint8 (wrapping every 256 send attempts, ~5.1 s at 50 Hz). Masking a uint8
    wrap with 32 bits yields ~4.29e9 instead of 1, which reads as a plausible
    huge number rather than an error -- so the width is a required part of
    getting this right, not a detail.
    """
    return (new - previous) & ((1 << width_bits) - 1)


def bind_udp_socket(
    device: str = '',
    address: str = '',
    port: int = 0,
    *,
    for_receive: bool = False,
    receive_buffer_bytes: Optional[int] = None,
    logger=None,
) -> socket.socket:
    """Create a UDP socket, optionally pinned to a device and bound to an address.

    ``device`` and ``address`` are both optional: when empty the socket keeps
    the kernel's default routing, which is what the fallbacks below fall back
    to. That is a legitimate configuration for a lab host with a single NIC,
    and it is also what happens here when the intended NIC is not present, so
    a failure to pin is logged and surfaced rather than swallowed.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    if device:
        try:
            sock.setsockopt(
                socket.SOL_SOCKET,
                SO_BINDTODEVICE,
                device.encode('utf-8'),
            )
            if logger:
                logger.info(f'UDP socket bound to device {device}')
        except OSError as exc:
            if logger:
                logger.error(
                    f'Failed to bind to device {device}: {exc}. '
                    'Using default routing.'
                )

    if address or port:
        try:
            sock.bind((address, port))
            if logger:
                logger.info(f'UDP socket bound to address {address}:{port}')
        except OSError as exc:
            if logger:
                logger.error(
                    f'Failed to bind to {address}:{port}: {exc}. '
                    'Using default routing.'
                )

    if receive_buffer_bytes:
        # A receive socket that is drained by a timer can be starved if the
        # timer is late, so give the kernel somewhere to put the backlog.
        # Failure to widen it is not fatal: the default is still workable at
        # the ~2.4 kB/s this link carries.
        try:
            sock.setsockopt(
                socket.SOL_SOCKET, socket.SO_RCVBUF, receive_buffer_bytes
            )
        except OSError as exc:
            if logger:
                logger.warn(f'Could not set SO_RCVBUF: {exc}')

    if for_receive:
        # Drained from a timer callback rather than a blocking reader, so the
        # socket must never block the executor.
        sock.setblocking(False)

    return sock
