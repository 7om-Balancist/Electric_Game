#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""台架测试工具：向 MSPM0 的视觉串口发送 12 字节位置帧。

用途
----
BUJIN 工程的视觉链路需要 MaixCAM 按固定协议发帧。在没有相机、或者相机端
发送程序还没写好时，用这个脚本从 PC（USB-TTL 转串口）直接喂帧，就能把
MSPM0 这一侧完整跑起来并验证故障保护。

接线
----
    PC USB-TTL TX  ->  MSPM0 PA31 (UART_MaixCAM RX)
    PC USB-TTL RX  ->  MSPM0 PA28 (UART_MaixCAM TX)
    GND            ->  GND
注意：MSPM0 的 PA28/PA31 目前和 OLED 的软件 I2C 冲突（详见 README），
先把 OLED 相关的引脚问题解决，否则 PA31 收不到任何东西。

帧格式（与 vision_link.h 一致）
-----------------------------
    offset 0   0xA5
    offset 1   0x5A
    offset 2   sequence   uint8，每帧 +1
    offset 3   flags      bit0 = 位置有效
    offset 4   x_px       int16 小端，0~319
    offset 6   camera_ms  uint16 小端
    offset 8   confidence uint8
    offset 9   reserved   0
    offset 10  crc16      小端，CRC-16/CCITT-FALSE（poly 0x1021, init 0xFFFF）
                          over 前 10 字节

常用模式
--------
    # 60Hz 正常发，小球在中心附近小幅摆动
    python vision_frame_sender.py --port COM5 --mode normal

    # 固定送 x=160（中心），用来标定"球在中心时机构是否水平"
    python vision_frame_sender.py --port COM5 --mode fixed --x 160

    # 模拟丢球：发 CRC 正确但 flags=0 的帧 -> MSPM0 应锁存 BALL_LOST (F8)
    python vision_frame_sender.py --port COM5 --mode lost

    # 模拟相机卡死：序号不递增 -> F8 同样应触发（这正是修复的那条判据）
    python vision_frame_sender.py --port COM5 --mode stuck-seq

    # 模拟 CRC 错误 -> MSPM0 应统计到 crc_errors，且不应更新位置
    python vision_frame_sender.py --port COM5 --mode badcrc

依赖：pyserial（pip install pyserial）
"""

import argparse
import math
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover
    print("需要 pyserial：pip install pyserial", file=sys.stderr)
    raise SystemExit(2)

FRAME_SIZE = 12
HEADER = (0xA5, 0x5A)
FLAG_VALID = 0x01
IMAGE_CENTER = 160


def crc16_ccitt_false(data: bytes) -> int:
    """与 MSPM0 端 crc16_ccitt() 完全一致的实现。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte & 0xFF) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(sequence: int, x_px: int, flags: int, camera_ms: int,
                confidence: int, corrupt_crc: bool = False) -> bytes:
    body = bytearray(FRAME_SIZE)
    body[0] = HEADER[0]
    body[1] = HEADER[1]
    body[2] = sequence & 0xFF
    body[3] = flags & 0xFF
    # int16 小端
    value = int(x_px) & 0xFFFF
    body[4] = value & 0xFF
    body[5] = (value >> 8) & 0xFF
    body[6] = camera_ms & 0xFF
    body[7] = (camera_ms >> 8) & 0xFF
    body[8] = confidence & 0xFF
    body[9] = 0x00
    crc = crc16_ccitt_false(bytes(body[:10]))
    if corrupt_crc:
        crc ^= 0x00FF
    body[10] = crc & 0xFF
    body[11] = (crc >> 8) & 0xFF
    return bytes(body)


def simulate_x(elapsed_s: float, amplitude: float, period_s: float) -> int:
    """一个缓慢的正弦摆动，用来代替真实小球运动。"""
    phase = 2.0 * math.pi * elapsed_s / max(0.1, period_s)
    return int(round(IMAGE_CENTER + amplitude * math.sin(phase)))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="向 MSPM0 视觉串口发送 12 字节位置帧（台架测试用）")
    parser.add_argument("--port", required=True, help="串口号，例如 COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=60.0, help="帧率 Hz")
    parser.add_argument("--mode", default="normal",
                        choices=["normal", "fixed", "lost", "stuck-seq",
                                 "badcrc", "slow"],
                        help="normal=正弦摆动 fixed=固定位置 lost=flags 全 0 "
                             "stuck-seq=序号不递增 badcrc=CRC 故意错 slow=1Hz")
    parser.add_argument("--x", type=int, default=IMAGE_CENTER,
                        help="fixed 模式下的像素坐标")
    parser.add_argument("--amplitude", type=float, default=60.0,
                        help="normal 模式的正弦幅度（像素）")
    parser.add_argument("--period", type=float, default=2.0,
                        help="normal 模式的正弦周期（秒）")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="运行时长，0 表示一直发直到 Ctrl+C")
    args = parser.parse_args()

    rate = 1.0 if args.mode == "slow" else max(1.0, args.rate)
    interval = 1.0 / rate

    sequence = 0
    sent = 0
    start = time.perf_counter()
    next_report = start + 1.0

    try:
        with serial.Serial(args.port, args.baud, timeout=0) as port:
            print("已打开 %s @ %d，模式=%s，帧率=%.1fHz，Ctrl+C 退出"
                  % (args.port, args.baud, args.mode, rate))
            while True:
                now = time.perf_counter()
                elapsed = now - start
                if args.seconds > 0 and elapsed >= args.seconds:
                    break

                if args.mode == "normal" or args.mode == "slow":
                    x_px = simulate_x(elapsed, args.amplitude, args.period)
                    flags = FLAG_VALID
                elif args.mode == "fixed":
                    x_px = args.x
                    flags = FLAG_VALID
                elif args.mode == "lost":
                    x_px = args.x
                    flags = 0x00          # 相机在发帧，但没有有效位置
                elif args.mode == "stuck-seq":
                    x_px = args.x
                    flags = FLAG_VALID
                else:  # badcrc
                    x_px = args.x
                    flags = FLAG_VALID

                frame = build_frame(
                    sequence=sequence,
                    x_px=x_px,
                    flags=flags,
                    camera_ms=int(elapsed * 1000.0) & 0xFFFF,
                    confidence=255,
                    corrupt_crc=(args.mode == "badcrc"),
                )
                port.write(frame)
                sent += 1

                # stuck-seq 模式刻意不递增序号，模拟相机端计数卡死
                if args.mode != "stuck-seq":
                    sequence = (sequence + 1) & 0xFF

                if now >= next_report:
                    print("已发送 %d 帧，seq=%d，x=%d，flags=0x%02X"
                          % (sent, sequence, x_px, flags))
                    next_report = now + 1.0

                sleep_for = interval - (time.perf_counter() - now)
                if sleep_for > 0:
                    time.sleep(sleep_for)
    except KeyboardInterrupt:
        print("\n已停止。")
    except serial.SerialException as exc:
        print("串口错误：%s" % exc, file=sys.stderr)
        return 1

    print("共发送 %d 帧。" % sent)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
