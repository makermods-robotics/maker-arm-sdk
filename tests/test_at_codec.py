from maker_arm.transport.at_serial import AtFrameParser, at_encode


def test_encode_golden():
    # enable motor 1: can_id=0x0300FD01 -> addr=(id<<3)|4=0x1807E80C
    frame = at_encode(0x0300FD01, bytes(8))
    assert frame == bytes.fromhex("4154" + "1807E80C" + "08" + "00" * 8 + "0D0A")


def test_parser_roundtrip_and_fragmentation():
    p = AtFrameParser()
    f1 = at_encode(0x0300FD01, bytes(8))
    f2 = at_encode(0x028001FD, bytes.fromhex("800080008000" + "0159"))
    stream = b"\x00garbage" + f1 + f2[:5]      # garbage prefix + partial packet
    out = p.feed(stream)
    assert out == [(0x0300FD01, bytes(8))]
    out = p.feed(f2[5:])                        # feed the rest
    assert out == [(0x028001FD, bytes.fromhex("8000800080000159"))]


def test_parser_resyncs_on_corrupt_frame():
    p = AtFrameParser()
    bad = bytearray(at_encode(0x0300FD01, bytes(8)))
    bad[-1] = 0x00                              # corrupt the frame tail
    good = at_encode(0x0400FD02, b"\x01")
    out = p.feed(bytes(bad) + good)
    assert out == [(0x0400FD02, b"\x01")]
