import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
from switch_protocol import classify_reply, mit_alive, private_alive, switch_acked  # noqa: E402


def test_classify_reply():
    assert classify_reply(0x02800307) == "private"   # 扩展帧（私有反馈）
    assert classify_reply(0x0FD) == "mit"            # 11 位标准帧
    assert classify_reply(0x007) == "mit"


def test_private_alive():
    replies = [(0x028003FD, bytes(8))]               # comm=2, motor_id=3（Bit8~15）
    assert private_alive(replies, 3) and not private_alive(replies, 4)


def test_mit_alive():
    # MIT 应答指令1：到主机 id 的标准帧，payload[0]=电机 id
    replies = [(0x0FD, bytes([5]) + bytes(7))]
    assert mit_alive(replies, 5) and not mit_alive(replies, 3)
    assert not mit_alive([(0x028005FD, bytes([5]) + bytes(7))], 5)  # 扩展帧不算


def test_switch_acked_accepts_experiment_frames():
    mcu = bytes.fromhex("7f5e38000c72b106")
    assert switch_acked([(0x7FE, mcu)], 7)      # 私有→MIT 的 Type0 应答（实测）
    assert switch_acked([(0x007, mcu)], 7)      # MIT→私有 指令8 应答（实测）


def test_switch_acked_rejects_noise():
    assert not switch_acked([], 7)
    assert not switch_acked([(0x028003FD, bytes(8))], 7)          # 别电机私有反馈
    assert not switch_acked([(0x0FD, bytes([3]) + bytes(7))], 7)  # 别电机 MIT 状态帧
    assert not switch_acked([(0x007, bytes(4))], 7)               # 载荷长度不对
