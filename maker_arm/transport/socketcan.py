"""Native AF_CAN socket backend (no python-can dependency). The Linux kernel CAN frame is a fixed 16 bytes."""

import logging
import socket
import struct
import threading
from typing import Callable, Optional

from .base import CanBackend

CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_ERR_FLAG = 0x20000000


def _frame_id(can_id: int, extended: bool) -> int:
    return (can_id | CAN_EFF_FLAG) if extended else (can_id & 0x7FF)


_FRAME = struct.Struct("=IB3x8s")  # can_id | dlc | pad | data
log = logging.getLogger("maker_arm.transport")


class SocketCanBackend(CanBackend):
    def __init__(self, channel: str = "can0"):
        self._channel = channel
        self._sock: Optional[socket.socket] = None
        self._cb: Optional[Callable[[int, bytes], None]] = None
        self._tx_lock = threading.Lock()
        self._running = False
        self._rx_thread: Optional[threading.Thread] = None

    def open(self) -> None:
        self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._sock.bind((self._channel,))
        self._sock.settimeout(0.1)
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True,
                                           name=f"socketcan-rx-{self._channel}")
        self._rx_thread.start()

    def close(self) -> None:
        self._running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        if self._sock:
            self._sock.close()
            self._sock = None

    def send(self, can_id: int, data: bytes, extended: bool = True) -> None:
        frame = _FRAME.pack(_frame_id(can_id, extended), len(data), data.ljust(8, b"\x00"))
        with self._tx_lock:
            self._sock.send(frame)

    def set_recv_callback(self, cb) -> None:
        self._cb = cb

    def _rx_loop(self) -> None:
        while self._running:
            try:
                buf = self._sock.recv(16)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                cid, dlc, payload = _FRAME.unpack(buf)
                if cid & CAN_ERR_FLAG:
                    continue
                cb = self._cb
                if cb:
                    rid = (cid & CAN_EFF_MASK) if (cid & CAN_EFF_FLAG) else (cid & 0x7FF)
                    cb(rid, payload[:dlc])
            except Exception:
                log.exception("socketcan RX frame-processing exception (continuing to receive frames)")
