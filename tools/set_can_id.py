#!/usr/bin/env python3
"""给电机分配 CAN ID（出厂默认 127）。

⚠️ 同一时刻总线上只能接【一个】待改 ID 的电机——SET_CAN_ID 按当前 ID 寻址，
多个电机同为 127 时会被一起改掉。标准流程：只接一个电机 → 分配 → 断开接下一个。

用法: python tools/set_can_id.py --current-id 127 --new-id 1
帧格式（协议 Type 7）: ID Bit23~16=新ID, Bit15~8=主机ID, Bit7~0=当前ID; data[0]=1。
"""

import argparse
import time

from maker_arm import protocol as p
from maker_arm.transport.base import create_backend


def probe(be, found, motor_id, tries=3):
    """发停止帧探测某 ID，返回是否收到反馈。"""
    found.clear()
    for _ in range(tries):
        be.send(*p.encode_disable(motor_id))
        time.sleep(0.05)
    return motor_id in found


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=["socketcan", "at"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--current-id", type=int, required=True)
    ap.add_argument("--new-id", type=int, required=True)
    a = ap.parse_args()
    if not (1 <= a.new_id <= 127):
        raise SystemExit("--new-id 必须在 1~127")

    kw = {"channel": a.channel} if a.backend == "socketcan" else {"port": a.port}
    be = create_backend(a.backend, **kw)
    found = {}
    be.set_recv_callback(lambda cid, d: (
        (msg := p.parse_frame(cid, d)) and isinstance(msg, p.MotorFeedback)
        and found.__setitem__(msg.motor_id, msg)))
    be.open()
    try:
        if not probe(be, found, a.current_id):
            raise SystemExit(f"当前 ID {a.current_id} 无反馈——查 ID 是否正确/电机是否上电")
        if probe(be, found, a.new_id):
            raise SystemExit(f"新 ID {a.new_id} 已被占用——先断开那个电机或换 ID")
        input(f"确认总线上只有这一个电机（ID {a.current_id}→{a.new_id}），回车执行 > ")

        # Type 7: data2 = (新ID << 8) | 主机ID; data[0]=1
        cid = p.make_can_id(p.COMM_SET_CAN_ID, (a.new_id << 8) | p.HOST_CAN_ID, a.current_id)
        data = bytearray(8)
        data[0] = 1
        be.send(cid, bytes(data))
        time.sleep(0.3)

        if not probe(be, found, a.new_id):
            raise SystemExit(f"改 ID 后新 ID {a.new_id} 探测失败——重试或用上位机核对")
        be.send(*p.encode_save_params(a.new_id))   # 掉电保存兜底（Type 7 本身即时生效）
        time.sleep(0.2)
        fb = found[a.new_id]
        print(f"✅ 电机现在是 ID {a.new_id}（位置 {fb.position:+.3f} rad，{fb.temperature:.1f}°C），已发掉电保存")
    finally:
        be.close()


if __name__ == "__main__":
    main()
