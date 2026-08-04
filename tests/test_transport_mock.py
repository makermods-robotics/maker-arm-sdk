from maker_arm.transport.base import CanBackend, create_backend
from maker_arm.transport.mock import MockBackend


def test_mock_records_and_injects():
    be = create_backend("mock")
    assert isinstance(be, CanBackend) and isinstance(be, MockBackend)
    got = []
    be.set_recv_callback(lambda cid, d: got.append((cid, d)))
    be.open()
    be.send(0x123, b"\x01\x02")
    assert be.sent == [(0x123, b"\x01\x02")]
    be.inject(0x456, b"\xAA")
    assert got == [(0x456, b"\xAA")]


def test_mock_responder_fires_synchronously():
    be = MockBackend()
    got = []
    be.set_recv_callback(lambda cid, d: got.append(cid))
    be.responder = lambda cid, d: [(cid + 1, b"")]
    be.open()
    be.send(10, b"")
    assert got == [11]
