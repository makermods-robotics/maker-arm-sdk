"""Minimal FashionStar sensor reader: only command 16 (query multi-turn angle).

Protocol reference:
https://github.com/motorbridge/motorbridge-smart-servo/blob/main/smart_servo_vendors/fashionstar/src/protocol.rs

The installed MotorBridge 0.0.4 API conflates held/filtered near-zero angles and
missing replies in its `reliable` flag. This diagnostic needs actual packet
freshness and raw angles, including a genuine zero. Each reply is checked for
header, length, checksum, query code and servo ID. There is no actuator API.
"""
from dataclasses import dataclass
import struct
import time

import serial


@dataclass(frozen=True)
class Reading:
    angle_deg: float
    reliable: bool = True


def angle_request(servo_id):
    if servo_id not in range(7):
        raise ValueError("This viewer reads only Star IDs 0–6")
    body = bytes([0x12, 0x4C, 16, 1, servo_id])
    return body + bytes([sum(body) & 0xFF])


def parse_angle(buffer, servo_id):
    """Consume complete packets; leave any trailing partial packet for next read."""
    while len(buffer) >= 5:
        if buffer[:2] != b"\x05\x1c":
            del buffer[0]
            continue
        size = buffer[3]
        if size > 64:  # Angle packets are tiny; reject a corrupt length early.
            del buffer[0]
            continue
        length = size + 5
        if len(buffer) < length:
            return None
        packet = bytes(buffer[:length])
        if sum(packet[:-1]) & 0xFF != packet[-1]:
            del buffer[0]
            continue
        del buffer[:length]
        if packet[2] == 16 and size >= 7 and packet[4] == servo_id:
            return Reading(struct.unpack_from("<i", packet, 5)[0] / 10.0)
    return None


class StarAngleBus:
    def __init__(self, port, baudrate=1_000_000):
        self.serial = serial.Serial(port, baudrate=baudrate, timeout=.003,
                                    write_timeout=.1, exclusive=True)

    def read_angle(self, servo_id):
        self.serial.reset_input_buffer()
        self.serial.write(angle_request(servo_id))
        buffer = bytearray()
        deadline = time.monotonic() + .035
        while time.monotonic() < deadline:
            buffer.extend(self.serial.read(max(1, self.serial.in_waiting)))
            sample = parse_angle(buffer, servo_id)
            if sample is not None:
                return sample
        return None

    def close(self):
        self.serial.close()
