"""Simulated MIT motors on a vcan interface, reusing the in-memory simulator from tests/fakes.py."""

from fakes import FakeMotor
from maker_arm.transport.socketcan import SocketCanBackend


class FakeMotorFleet:
    def __init__(self, channel: str, motor_ids: list[int]):
        self._be = SocketCanBackend(channel)
        self.motors = {mid: FakeMotor(mid) for mid in motor_ids}
        self._be.set_recv_callback(self._on_frame)

    @property
    def paused(self) -> bool:
        return not all(m.online for m in self.motors.values())

    @paused.setter
    def paused(self, value: bool) -> None:
        for m in self.motors.values():
            m.online = not value

    def start(self) -> None:
        self._be.open()

    def stop(self) -> None:
        self._be.close()

    def _on_frame(self, can_id, data) -> None:
        if getattr(can_id, "extended", False):
            return
        m = self.motors.get(can_id & 0x7F)
        if m is None or can_id == 0xFD:   # ignore our own replies
            return
        for cid, payload in m.handle(can_id, data):
            self._be.send(cid, payload, extended=False)
