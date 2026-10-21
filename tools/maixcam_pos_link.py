# -*- coding: utf-8 -*-
"""MaixCAM -> MSPM0 位置帧发送模块。

为什么要写这个文件
------------------
BUJIN 的 MSPM0 工程已经实现了 12 字节位置帧的接收端（vision_link.c），
但 MaixCAM 侧的 v6.12.37 工程里**没有对应的发送端**：它内部是"相机直接
驱动 ZDT 步进电机"（UART2 → ZDT），UART1 只用来**接收**主控发来的单字节
IMU 前馈。也就是说，12 字节协议只有一半。

把本文件放进 MaixCAM 工程目录，按下面的片段接进 main.py 即可补齐发送端。

帧格式（必须与 MSPM0 的 vision_link.h 完全一致）
----------------------------------------------
    offset 0   0xA5
    offset 1   0x5A
    offset 2   sequence   uint8，每帧 +1（回绕到 0 是安全的）
    offset 3   flags      bit0 = 位置有效；丢球时清 0
    offset 4   x_px       int16 小端
    offset 6   camera_ms  uint16 小端
    offset 8   confidence uint8
    offset 9   reserved   0
    offset 10  crc16      小端，CRC-16/CCITT-FALSE
                          poly 0x1021, init 0xFFFF, 不反射, 不异或输出
                          计算范围是前 10 字节

接入 main.py（示意）
-------------------
    from maix import uart, pinmap, time
    from car_pos_link import CarPosLink

    # 选一对空闲的 UART 引脚，TX 接到 MSPM0 的 PA31(UART0_RX)
    for pin, function in {"A30": "UART1_TX", "A31": "UART1_RX"}.items():
        pinmap.set_pin_function(pin, function)
    pos_uart = uart.UART("/dev/ttyS1", 115200)
    self.pos_link = CarPosLink(pos_uart, time)

    # 每帧视觉结果出来后调用一次（内部按 tx_period_ms 限速，重复调用无副作用）
    self.pos_link.send(x_px=int(tracker["x"]),
                       valid=bool(tracker["valid"]),
                       confidence=int(tracker["confidence"]))

注意
----
1. 丢球时**必须继续发帧**并清掉 bit0，千万不要停止发送：
   MSPM0 端要区分"链路断了"和"球丢了"，前者报 VISION_LOST(F1)，
   后者报 BALL_LOST(F8)。停止发送会让两种故障混在一起。
2. 不要重复使用同一个 sequence：MSPM0 用序列号做去重令牌，序号不递增
   会被当成"没有新样本"，同样触发 BALL_LOST。
3. CRC 参数必须严格一致，否则 MSPM0 端会全部丢帧并累加 crc_errors。
"""

import threading

FRAME_SIZE = 12
HEADER_0 = 0xA5
HEADER_1 = 0x5A
FLAG_VALID = 0x01
DEFAULT_TX_PERIOD_MS = 16      # 约 60Hz，与相机的视觉帧率一致


def crc16_ccitt_false(data):
    """CRC-16/CCITT-FALSE，与 MSPM0 端 crc16_ccitt() 逐位等价。"""
    crc = 0xFFFF
    for byte in bytearray(data):
        crc ^= (int(byte) & 0xFF) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(sequence, x_px, flags, camera_ms, confidence):
    """打包一帧。返回 bytes，长度固定 12。"""
    body = bytearray(FRAME_SIZE)
    body[0] = HEADER_0
    body[1] = HEADER_1
    body[2] = int(sequence) & 0xFF
    body[3] = int(flags) & 0xFF
    value = int(x_px) & 0xFFFF          # 负数也要按补码发
    body[4] = value & 0xFF
    body[5] = (value >> 8) & 0xFF
    body[6] = int(camera_ms) & 0xFF
    body[7] = (int(camera_ms) >> 8) & 0xFF
    body[8] = int(confidence) & 0xFF
    body[9] = 0x00
    crc = crc16_ccitt_false(bytes(body[:10]))
    body[10] = crc & 0xFF
    body[11] = (crc >> 8) & 0xFF
    return bytes(body)


class CarPosLink:
    """把小球位置按固定协议发给 MSPM0，自带限速与统计。"""

    def __init__(self, serial, time_mod, tx_period_ms=DEFAULT_TX_PERIOD_MS):
        self.serial = serial
        self.time = time_mod
        self.tx_period_ms = max(0, int(tx_period_ms))
        self.lock = threading.Lock()
        self.sequence = 0
        self.tx_count = 0
        self.skip_count = 0
        self.error_count = 0
        self.last_tx_ms = -1000000
        self.last_x = 0
        self.last_flags = 0
        self.error = ""

    def send(self, x_px, valid=True, confidence=255, now_ms=None, force=False):
        """发送一帧；距离上一帧不足 tx_period_ms 时直接跳过。

        返回 True 表示真的发出去了。串口异常不会抛出，只累加 error_count，
        这样视觉主循环不会被一个坏掉的串口打断。
        """
        now = int(self.time.ticks_ms() if now_ms is None else now_ms)
        with self.lock:
            if (not force) and ((now - self.last_tx_ms) < self.tx_period_ms):
                self.skip_count += 1
                return False

            flags = FLAG_VALID if valid else 0x00
            frame = build_frame(
                sequence=self.sequence,
                x_px=x_px,
                flags=flags,
                camera_ms=now & 0xFFFF,
                confidence=confidence,
            )
            try:
                self.serial.write(frame)
            except Exception as exc:      # 串口异常不能影响视觉主循环
                self.error = "tx:" + repr(exc)
                self.error_count += 1
                return False

            self.sequence = (self.sequence + 1) & 0xFF
            self.tx_count += 1
            self.last_tx_ms = now
            self.last_x = int(x_px)
            self.last_flags = flags
            return True

    def snapshot(self):
        """给 session_logger / 调试界面用的一份快照。"""
        with self.lock:
            return {
                "tx_count": int(self.tx_count),
                "skip_count": int(self.skip_count),
                "error_count": int(self.error_count),
                "sequence": int(self.sequence),
                "last_x": int(self.last_x),
                "last_flags": int(self.last_flags),
                "tx_period_ms": int(self.tx_period_ms),
                "error": str(self.error),
            }
