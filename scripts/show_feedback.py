#!/usr/bin/env python3
"""实时打印运控板回传的 sta__ 状态帧（56 字节，Version=1）。

只接收，不发送任何控制帧。字段含义与换算见 docs/上位机接口说明_STA56.md。

用法:
    ./scripts/show_feedback.py          # Ctrl-C 结束

有效位为 0 的字段打印 INVALID，不会拿线值的 0 冒充实测零值（接口说明 5.2）。
"""

import socket
import sys
from pathlib import Path

sys.path.insert(0, str(
    Path(__file__).resolve().parents[1] / 'src' / 'motion_interface'
))

from motion_interface.sta56 import (  # noqa: E402
    BRAKE_MODE_NAMES,
    CONTROL_SOURCE_NAMES,
    decode_status_frame,
    validate_status_frame,
)

# 板端在 192.168.5.0/24（雷达 192.168.1.0/24 走另一个网口 enp2s0）。
DEVICE = 'enp5s0'
ADDRESS = '192.168.5.11'
PORT = 5001                     # 状态 UDP 源/目标端口相同
BOARD_ADDRESS = '192.168.5.50'

SO_BINDTODEVICE = 25            # Linux 专用 socket 选项

GEAR_NAMES = {1: 'forward', -1: 'reverse', 0: 'stopped'}

# 转角两项都有效时应为 0x0C（bit2 换算角度 + bit3 原始角度），
# 这是现场判断 EPS 那一路通没通最快的单一信号。
EPS_MASK = 0x0C


def show(text, valid):
    """有效位清零时显示 INVALID。"""
    return text if valid else 'INVALID'


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, SO_BINDTODEVICE, DEVICE.encode())
    sock.bind((ADDRESS, PORT))
    print(f'监听 {ADDRESS}:{PORT}（网卡 {DEVICE}），只收不发，Ctrl-C 结束')

    while True:
        frame, source = sock.recvfrom(2048)

        reason = validate_status_frame(frame)
        if reason is not None:
            print(f'拒收({reason}) 来源{source[0]}:{source[1]} '
                  f'len={len(frame)} {frame.hex(" ").upper()}')
            continue

        f = decode_status_frame(frame)
        mask = f.valid_flags & EPS_MASK
        src_note = ('' if source[0] == BOARD_ADDRESS
                    else f'  来源异常!={source[0]}:{source[1]}')
        print(
            f'cnt={f.counter:3d} '
            f't={f.timestamp_ms}ms '
            f'speed={show(f"{f.actual_speed_mps:+.4f}", f.speed_valid)} m/s '
            f'steer={show(f"{f.actual_steering_deg:+.4f}", f.steering_valid)}'
            f' deg '
            f'raw={show(f"{f.steering_raw:+d}", f.steering_raw_valid)}'
            f'(÷10→{show(f"{f.steering_raw / 10:+.1f}", f.steering_raw_valid)}'
            f' deg) '
            f'EPS链路=0x{mask:02X}'
            f'{" OK" if mask == EPS_MASK else " (角度未收到)"} '
            f'brake={BRAKE_MODE_NAMES.get(f.brake_mode)}'
            f'/{f.brake_actual_text} '
            f'gear={GEAR_NAMES.get(f.gear, "?")} '
            f'src={CONTROL_SOURCE_NAMES.get(f.control_source)} '
            f'rt49=0x{f.rt49_status:04X} eps=0x{f.eps_status:04X} '
            f'seb=0x{f.seb_status:04X} '
            f'vflags=0x{f.valid_flags:02X} fault=0x{f.fault_flags:08X}'
            f'{src_note}'
        )


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
