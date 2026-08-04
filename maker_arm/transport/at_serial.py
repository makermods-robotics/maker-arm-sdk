"""灵足官方 USB-CAN 板（GD32+CH340）后端：串口 921600 8N1，AT 帧封装。

帧格式（收发同构）：b"AT" + ((can_id29<<3)|0x04) 大端4字节 + len + data(0~8) + b"\r\n"
参考 el_a3_sdk/at_can_driver.py（本机已实测 7/7 电机通信）。
"""

import logging
import threading
from typing import Callable, Optional

from .base import CanBackend

log = logging.getLogger("maker_arm.transport")


def at_encode(can_id: int, data: bytes) -> bytes:
    addr = ((can_id << 3) | 0x04) & 0xFFFFFFFF
    return b"AT" + addr.to_bytes(4, "big") + bytes([len(data)]) + data + b"\r\n"


class AtFrameParser:
    """字节流 → 帧。处理垃圾前缀、半包、坏帧重同步。"""

    def __init__(self):
        self._buf = b""

    def feed(self, chunk: bytes) -> list[tuple[int, bytes]]:
        self._buf += chunk
        out: list[tuple[int, bytes]] = []
        while True:
            i = self._buf.find(b"AT")
            if i < 0:
                self._buf = self._buf[-1:]      # 留 1 字节防 'A''T' 被拆开
                break
            self._buf = self._buf[i:]
            if len(self._buf) < 7:
                break
            dlc = self._buf[6]
            if dlc > 8:
                self._buf = self._buf[2:]        # 坏帧：跳过本 'AT' 重找
                continue
            end = 7 + dlc + 2
            if len(self._buf) < end:
                break
            if self._buf[end - 2:end] != b"\r\n":
                self._buf = self._buf[2:]
                continue
            addr = int.from_bytes(self._buf[2:6], "big")
            out.append((addr >> 3, self._buf[7:7 + dlc]))
            self._buf = self._buf[end:]
        return out


class AtSerialBackend(CanBackend):
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 921600):
        self._port, self._baud = port, baudrate
        self._ser = None
        self._cb: Optional[Callable[[int, bytes], None]] = None
        self._tx_lock = threading.Lock()
        self._running = False
        self._rx_thread: Optional[threading.Thread] = None

    def open(self) -> None:
        import serial
        self._ser = serial.Serial(self._port, self._baud, timeout=0.02)
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True,
                                           name=f"at-rx-{self._port}")
        self._rx_thread.start()

    def close(self) -> None:
        self._running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        if self._ser:
            self._ser.close()
            self._ser = None

    def send(self, can_id: int, data: bytes) -> None:
        with self._tx_lock:
            self._ser.write(at_encode(can_id, data))

    def set_recv_callback(self, cb) -> None:
        self._cb = cb

    def _rx_loop(self) -> None:
        parser = AtFrameParser()
        while self._running:
            try:
                chunk = self._ser.read(max(1, self._ser.in_waiting))
            except Exception:
                break
            if not chunk:
                continue
            try:
                cb = self._cb
                for cid, d in parser.feed(chunk):
                    if cb:
                        cb(cid, d)
            except Exception:
                log.exception("AT RX 帧处理异常（继续收帧）")
