"""Backend for testing / no hardware: records sends, allows injecting receives, and supports attaching an auto-responder."""

from typing import Callable, Optional

from .base import CanBackend


class MockBackend(CanBackend):
    def __init__(self):
        self.sent: list[tuple[int, bytes]] = []
        self.is_open = False
        self._cb: Optional[Callable[[int, bytes], None]] = None
        # responder(can_id, data) -> [(can_id, data), ...] to inject, or None
        self.responder: Optional[Callable] = None

    def open(self) -> None:
        self.is_open = True

    def close(self) -> None:
        self.is_open = False

    def send(self, can_id: int, data: bytes) -> None:
        self.sent.append((can_id, bytes(data)))
        if self.responder and self._cb:
            for cid, d in (self.responder(can_id, data) or []):
                self._cb(cid, d)

    def set_recv_callback(self, cb) -> None:
        self._cb = cb

    def inject(self, can_id: int, data: bytes) -> None:
        if self._cb:
            self._cb(can_id, data)
