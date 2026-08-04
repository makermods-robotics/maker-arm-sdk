"""通用 leader→follower 线性关节映射（不依赖具体 leader 硬件）。

公式：rad_i = base_rad_i + direction_i * scale_i * radians(deg_i - zero_deg_i)
输入角先做 EMA 平滑（首帧直通）；缺读关节保持上次输出。
标定：tools/calib_star_map.py 两姿势法生成 JSON。
"""

import json
import math


class JointMapper:
    def __init__(self, joints: list[dict], alpha: float = 0.3):
        self._joints = joints
        self._alpha = alpha
        self._smooth: list[float | None] = [None] * len(joints)
        self._last_out: list[float] = [j["base_rad"] for j in joints]

    @classmethod
    def from_json(cls, path: str) -> "JointMapper":
        with open(path) as f:
            raw = json.load(f)
        return cls(raw["joints"], alpha=raw.get("alpha", 0.3))

    def map(self, raw_deg: dict[int, float]) -> list[float]:
        out = list(self._last_out)
        for i, j in enumerate(self._joints):
            deg = raw_deg.get(j["servo"])
            if deg is None:
                continue
            if self._smooth[i] is None:
                self._smooth[i] = deg
            else:
                self._smooth[i] = self._alpha * deg + (1.0 - self._alpha) * self._smooth[i]
            out[i] = (j["base_rad"] + j["direction"] * j["scale"]
                      * math.radians(self._smooth[i] - j["zero_deg"]))
        self._last_out = out
        return out
