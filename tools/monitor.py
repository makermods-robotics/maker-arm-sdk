#!/usr/bin/env python3
"""只读监视 + 限位采集：CONNECTED 态 10Hz 轮询刷屏（未使能，可安全手推）。

手推各关节到两端极限后按 Enter：自动把回退后的 lo/hi 写入 YAML（保留注释，
写前备份 .bak，写后校验失败自动回滚），并存 configs/limits_capture.json 原始记录。
Ctrl-C 退出则什么都不写。
"""

import argparse
import json
import math
import re
import select
import shutil
import sys
import time
from datetime import datetime

from maker_arm.arm import Arm
from maker_arm.config import ArmConfig
from maker_arm.errors import fault_text

MIN_TRAVEL = 0.3  # rad：行程小于此视为该关节没测过，跳过写入


def compute_limits(mins, maxs, backoff, min_travel=MIN_TRAVEL):
    """逐关节返回 (lo, hi) 或 None（无数据/行程不足/回退后区间塌缩）。"""
    out = []
    for lo_raw, hi_raw in zip(mins, maxs):
        if not (math.isfinite(lo_raw) and math.isfinite(hi_raw)):
            out.append(None)
            continue
        if hi_raw - lo_raw < min_travel:
            out.append(None)
            continue
        lo, hi = round(lo_raw + backoff, 3), round(hi_raw - backoff, 3)
        out.append((lo, hi) if lo < hi else None)
    return out


def update_yaml_limits(text: str, per_motor: dict) -> str:
    """逐行替换 motor_id 对应行内的 lo:/hi: 数值，注释与格式原样保留。"""
    lines = text.splitlines(keepends=True)
    for i, line in enumerate(lines):
        m = re.search(r"motor_id:\s*(\d+)", line)
        if not m or int(m.group(1)) not in per_motor:
            continue
        lo, hi = per_motor[int(m.group(1))]
        line = re.sub(r"(lo:\s*)-?\d+(?:\.\d+)?", lambda g: f"{g.group(1)}{lo}", line, count=1)
        line = re.sub(r"(hi:\s*)-?\d+(?:\.\d+)?", lambda g: f"{g.group(1)}{hi}", line, count=1)
        lines[i] = line
    return "".join(lines)


def write_limits(config_path, json_path, mins, maxs, backoff, joints):
    limits = compute_limits(mins, maxs, backoff)
    per_motor, record_joints = {}, []
    for i, j in enumerate(joints):
        entry = {"joint": i + 1, "motor_id": j.motor_id,
                 "min": round(mins[i], 4) if math.isfinite(mins[i]) else None,
                 "max": round(maxs[i], 4) if math.isfinite(maxs[i]) else None}
        if limits[i] is None:
            entry.update(written=False, reason="无数据或行程不足（未手推到两端？）")
            print(f"⚠️ J{i + 1} (电机 {j.motor_id}) 跳过：{entry['reason']}")
        else:
            per_motor[j.motor_id] = limits[i]
            entry.update(written=True, lo=limits[i][0], hi=limits[i][1])
            print(f"J{i + 1} (电机 {j.motor_id}): lo={limits[i][0]} hi={limits[i][1]}")
        record_joints.append(entry)

    if per_motor:
        bak = config_path + ".bak"
        shutil.copy2(config_path, bak)
        with open(config_path) as f:
            text = f.read()
        with open(config_path, "w") as f:
            f.write(update_yaml_limits(text, per_motor))
        try:
            ArmConfig.from_yaml(config_path)
        except Exception as e:
            shutil.copy2(bak, config_path)
            raise SystemExit(f"写入后校验失败，已从 {bak} 回滚：{e}")
        print(f"已写入 {config_path}（备份在 {bak}）")
    else:
        print("没有任何关节达到写入条件，YAML 未改动")

    with open(json_path, "w") as f:
        json.dump({"timestamp": datetime.now().isoformat(timespec="seconds"),
                   "backoff": backoff, "min_travel": MIN_TRAVEL,
                   "joints": record_joints}, f, ensure_ascii=False, indent=2)
    print(f"原始记录已存 {json_path}")


def _fmt(x):
    return f"{x:+7.3f}" if math.isfinite(x) else "  ——   "


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="configs/maker_arm_6dof.yaml")
    ap.add_argument("--backend", choices=["socketcan", "at"], default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--backoff", type=float, default=0.05, help="限位回退量 rad")
    ap.add_argument("--json", dest="json_path", default="configs/limits_capture.json")
    a = ap.parse_args()

    kw = {"channel": a.channel} if a.backend == "socketcan" else {"port": a.port}
    arm = Arm.from_yaml(a.config, backend=a.backend, **kw)
    arm.connect()
    n = arm.config.n_joints
    mins, maxs = [math.inf] * n, [-math.inf] * n
    print("已连接（未使能，可安全手推）。")
    try:
        while True:
            arm.refresh()
            time.sleep(0.1)
            pos, vel = arm.get_joint_positions(), arm.get_joint_velocities()
            tmp, flt = arm.get_temperatures(), arm.get_faults()
            for i, p in enumerate(pos):
                if math.isfinite(p):
                    mins[i] = min(mins[i], p)
                    maxs[i] = max(maxs[i], p)
            rows = [f"J{i+1}: {pos[i]:+7.3f} rad {vel[i]:+6.2f} rad/s {tmp[i]:5.1f}°C "
                    f"min {_fmt(mins[i])} max {_fmt(maxs[i])} {fault_text(flt[i])}"
                    for i in range(n)]
            rows.append("—— 手推各关节到两端极限。按 Enter 写入限位并退出；Ctrl-C 退出不写。")
            print("\x1b[2J\x1b[H" + "\n".join(rows), flush=True)
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                print()
                write_limits(a.config, a.json_path, mins, maxs, a.backoff, arm.config.joints)
                break
    except KeyboardInterrupt:
        print("\n未写入任何文件")
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
