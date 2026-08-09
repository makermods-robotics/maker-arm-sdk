import os
import time

import pytest

from maker_arm.transport.socketcan import SocketCanBackend

VCAN_UP = os.path.exists("/sys/class/net/vcan0")
pytestmark = pytest.mark.skipif(not VCAN_UP, reason="vcan0 does not exist (sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0)")


def test_loopback_two_sockets():
    a, b = SocketCanBackend("vcan0"), SocketCanBackend("vcan0")
    got = []
    b.set_recv_callback(lambda cid, d: got.append((cid, d)))
    a.open(); b.open()
    try:
        a.send(0x0300FD01, bytes(8))
        deadline = time.monotonic() + 1.0
        while not got and time.monotonic() < deadline:
            time.sleep(0.01)
        assert got and got[0][0] == 0x0300FD01 and got[0][1] == bytes(8)
    finally:
        a.close(); b.close()


def test_loopback_standard_frame():
    a, b = SocketCanBackend("vcan0"), SocketCanBackend("vcan0")
    got = []
    b.set_recv_callback(lambda cid, d: got.append((cid, d)))
    a.open(); b.open()
    try:
        a.send(0x007, bytes.fromhex("FFFFFFFFFFFF00FD"), extended=False)
        deadline = time.monotonic() + 1.0
        while not got and time.monotonic() < deadline:
            time.sleep(0.01)
        assert got and got[0][0] == 0x007 and got[0][1].hex() == "ffffffffffff00fd"
    finally:
        a.close(); b.close()
