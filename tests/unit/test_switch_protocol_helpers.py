from tools.experimental.lerobot.switch_protocol import (
    classify_reply,
    mit_alive,
    private_alive,
    switch_acked,
)


def test_classify_reply():
    assert classify_reply(0x02800307) == "private"   # extended frame (private feedback)
    assert classify_reply(0x0FD) == "mit"            # 11-bit standard frame
    assert classify_reply(0x007) == "mit"


def test_private_alive():
    replies = [(0x028003FD, bytes(8))]               # comm=2, motor_id=3 (Bit8~15)
    assert private_alive(replies, 3) and not private_alive(replies, 4)


def test_mit_alive():
    # MIT reply command 1: standard frame addressed to the host id, payload[0]=motor id
    replies = [(0x0FD, bytes([5]) + bytes(7))]
    assert mit_alive(replies, 5) and not mit_alive(replies, 3)
    assert not mit_alive([(0x028005FD, bytes([5]) + bytes(7))], 5)  # extended frame doesn't count


def test_switch_acked_accepts_experiment_frames():
    mcu = bytes.fromhex("7f5e38000c72b106")
    assert switch_acked([(0x7FE, mcu)], 7)      # private->MIT Type0 reply (7<<8)|0xFE (measured on real hardware)
    assert switch_acked([(0x7FFE, mcu)], 127)   # same shape, motor 127 (measured 2026-08-07)
    assert switch_acked([(0x007, mcu)], 7)      # MIT->private command 8 reply (measured on real hardware)
    assert not switch_acked([(0x7FE, mcu)], 8)  # a Type0 reply for a different motor is not accepted


def test_switch_acked_rejects_noise():
    assert not switch_acked([], 7)
    assert not switch_acked([(0x028003FD, bytes(8))], 7)          # private feedback for a different motor
    assert not switch_acked([(0x0FD, bytes([3]) + bytes(7))], 7)  # MIT status frame for a different motor
    assert not switch_acked([(0x007, bytes(4))], 7)               # wrong payload length
    assert not switch_acked([(0x003, bytes(8))], 7)               # unrelated 8-byte frame (outside the whitelist)
    assert switch_acked([(0x0FD, bytes([7]) + bytes(7))], 7)      # addressed to host and payload[0]=self -> accepted
