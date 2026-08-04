"""vcan 上的假电机群：足够以假乱真地测 Arm 全链路，不含物理模型。"""

import struct

from maker_arm import protocol as p
from maker_arm.transport.socketcan import SocketCanBackend


class FakeMotorFleet:
    def __init__(self, channel: str, motor_ids: list[int]):
        self._be = SocketCanBackend(channel)
        self.positions = {mid: 0.0 for mid in motor_ids}   # 电机坐标 rad
        self.paused = False
        self._be.set_recv_callback(self._on_frame)

    def start(self) -> None:
        self._be.open()

    def stop(self) -> None:
        self._be.close()

    def _feedback(self, mid: int) -> tuple[int, bytes]:
        cid = (p.COMM_FEEDBACK << 24) | (2 << 22) | (mid << 8) | p.HOST_CAN_ID
        data = struct.pack(">HHHH",
                           p.float_to_u16(self.positions[mid], p.P_MIN, p.P_MAX),
                           p.float_to_u16(0.0, p.V_MIN, p.V_MAX),
                           p.float_to_u16(0.0, p.T_MIN, p.T_MAX), 300)
        return cid, data

    def _on_frame(self, can_id: int, data: bytes) -> None:
        mid = can_id & 0xFF
        if mid not in self.positions or self.paused:
            return
        comm = (can_id >> 24) & 0x1F
        if comm == p.COMM_MIT:
            self.positions[mid] = p.u16_to_float(
                int.from_bytes(data[:2], "big"), p.P_MIN, p.P_MAX)
            self._be.send(*self._feedback(mid))
        elif comm in (p.COMM_ENABLE, p.COMM_DISABLE, p.COMM_SET_ZERO):
            self._be.send(*self._feedback(mid))
        elif comm == p.COMM_READ_PARAM:
            idx = struct.unpack("<H", data[:2])[0]
            cid = (p.COMM_READ_PARAM << 24) | (mid << 8) | p.HOST_CAN_ID
            self._be.send(cid, struct.pack("<H2x", idx) + struct.pack("<f", 0.0))
