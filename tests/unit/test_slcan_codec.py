import pytest

from maker_arm.transport.slcan_serial import SlcanFrameParser, SlcanSerialBackend, slcan_encode


def test_encode_extended_and_standard_frames():
    assert slcan_encode(0x0400FD01, bytes(8)) == b"T0400FD0180000000000000000\r"
    assert slcan_encode(0x123, b"\xAB\xCD", extended=False) == b"t1232ABCD\r"


def test_parser_fragmentation_ack_garbage_and_timestamp():
    parser = SlcanFrameParser()
    timestamped = slcan_encode(0x028001FD, bytes.fromhex("8000800080000159"))[:-1] + b"1234\r"
    stream = b"\rgarbage\r" + slcan_encode(0x0400FD01, bytes(8)) + timestamped
    assert parser.feed(stream[:19]) == []
    assert parser.feed(stream[19:]) == [
        (0x0400FD01, bytes(8)),
        (0x028001FD, bytes.fromhex("8000800080000159")),
    ]


def test_parser_recovers_after_malformed_records():
    parser = SlcanFrameParser()
    assert parser.feed(b"T00000001900\rTZZZZZZZZ1AA\r" + slcan_encode(0x101, b"\x01")) == [
        (0x101, b"\x01")
    ]


def test_encode_rejects_invalid_id_and_payload():
    with pytest.raises(ValueError):
        slcan_encode(0x20000000, b"")
    with pytest.raises(ValueError):
        slcan_encode(0x800, b"", extended=False)
    with pytest.raises(ValueError):
        slcan_encode(1, bytes(9))


@pytest.mark.parametrize(("auto_retransmit", "command"), [
    (False, b"C\rS8\rA0\rO\r"),
    (True, b"C\rS8\rA1\rO\r"),
])
def test_backend_configures_canable_retry_mode(monkeypatch, auto_retransmit, command):
    import serial

    class FakeSerial:
        def __init__(self, *args, **kwargs):
            self.writes = []
            self.in_waiting = 0

        def reset_input_buffer(self):
            pass

        def write(self, data):
            self.writes.append(data)

        def flush(self):
            pass

        def read(self, _size):
            return b""

        def close(self):
            pass

    fake = FakeSerial()
    opened = {}
    def open_serial(*args, **kwargs):
        opened["args"] = args
        opened["kwargs"] = kwargs
        return fake
    monkeypatch.setattr(serial, "Serial", open_serial)
    backend = SlcanSerialBackend(startup_delay=0, auto_retransmit=auto_retransmit)
    backend.open()
    backend.close()
    assert opened["args"][:2] == ("/dev/ttyUSB0", 115200)
    assert opened["kwargs"]["rtscts"] is False
    assert fake.writes == [command, b"C\r"]
