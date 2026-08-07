#!/usr/bin/env python3
"""电机通信协议双向切换（私有 ↔ MIT）。切换后必须断电重启才生效。

私有=维护态（maker-arm SDK/工具），MIT=lerobot 运行态（RobstrideMotorsBus）。
每台电机先探测当前协议再动作；帧格式 2026-08-07 真机实测锚定。仅支持 socketcan。
"""

import argparse
import time

from maker_arm import protocol as p
from maker_arm.transport.socketcan import SocketCanBackend

PROBE_WAIT = 0.3


def classify_reply(can_id: int) -> str:
    """>0x7FF 只可能是 29 位扩展帧（私有协议应答）；否则是 11 位标准帧（MIT 应答）。"""
    return "private" if can_id > 0x7FF else "mit"


def private_alive(replies: list, motor_id: int) -> bool:
    return any(classify_reply(cid) == "private"
               and (cid >> 24) & 0x1F == p.COMM_FEEDBACK
               and (cid >> 8) & 0xFF == motor_id for cid, _ in replies)


def mit_alive(replies: list, motor_id: int, host_id: int = p.HOST_CAN_ID) -> bool:
    return any(classify_reply(cid) == "mit" and cid == host_id
               and len(d) >= 1 and d[0] == motor_id for cid, d in replies)


def switch_acked(replies: list, motor_id: int) -> bool:
    """切换指令的应答判定（设备ID/MCU码帧，8 字节载荷）。

    实测（2026-08-07）：私有→MIT 的 Type0 应答 cid=0x7FE；MIT→私有的指令8应答
    cid=电机id——两者载荷都是 8 字节 MCU 唯一码。排除两类误认：普通私有反馈帧、
    发给主机但属于其它电机的 MIT 状态帧。
    """
    for cid, d in replies:
        if len(d) != 8:
            continue
        if classify_reply(cid) == "private" and (cid >> 24) & 0x1F == p.COMM_FEEDBACK:
            continue
        if cid == p.HOST_CAN_ID and (len(d) < 1 or d[0] != motor_id):
            continue
        return True
    return False


def detect(be, replies: list, motor_id: int) -> str | None:
    """探测电机当前协议：分别发私有停止帧与 MIT 只读故障查询，看谁有应答。"""
    replies.clear()
    be.send(*p.encode_disable(motor_id))                          # 私有探测（扩展帧）
    time.sleep(PROBE_WAIT)
    if private_alive(list(replies), motor_id):
        return "private"
    replies.clear()
    be.send(motor_id, p.mit_fault_query_data(), extended=False)   # MIT 探测（标准帧，无副作用）
    time.sleep(PROBE_WAIT)
    if mit_alive(list(replies), motor_id):
        return "mit"
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--ids", required=True, help="逗号分隔电机 ID，如 3,4,5,6,7")
    ap.add_argument("--to", choices=["mit", "private"], required=True)
    a = ap.parse_args()
    ids = [int(x) for x in a.ids.split(",")]

    replies: list = []
    be = SocketCanBackend(a.channel)
    be.set_recv_callback(lambda cid, d: replies.append((cid, bytes(d))))
    be.open()
    switched, skipped, failed = [], [], []
    try:
        for mid in ids:
            cur = detect(be, replies, mid)
            if cur is None:
                failed.append(mid)
                print(f"电机{mid}: ❌ 两种协议都无应答——查电源/接线/是否已切换未重启")
                continue
            if cur == a.to:
                skipped.append(mid)
                print(f"电机{mid}: 已是 {cur}，跳过")
                continue
            replies.clear()
            if a.to == "mit":
                be.send(*p.encode_set_protocol(mid, 2))            # 私有 Type25 → MIT
            else:
                be.send(mid, p.mit_switch_protocol_data(0), extended=False)  # MIT 指令8 → 私有
            time.sleep(0.5)
            if switch_acked(list(replies), mid):
                switched.append(mid)
                print(f"电机{mid}: {cur} → {a.to} 指令已应答 ✅")
            else:
                failed.append(mid)
                print(f"电机{mid}: ⚠️ 切换指令无应答——重试或核对协议状态")
    finally:
        be.close()
    print(f"\n切换 {switched}，跳过 {skipped}，失败 {failed}")
    if switched:
        print("⚠️ 重新上电后才生效：请给电机断电重启，然后验证——"
              f"{'python tools/scan_bus.py（应无应答）' if a.to == 'mit' else 'python tools/scan_bus.py（应全在线）'}")
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()
