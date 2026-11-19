"""Simulated motors on a vcan interface, reusing the in-memory simulator from tests/fakes.py."""

from conftest import sdk
from fakes import FakeMotor, respond

SocketCanBackend = sdk.transport.socketcan.SocketCanBackend


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
        for cid, payload in respond(self.motors, can_id, data):
            self._be.send(cid, payload, extended=False)
