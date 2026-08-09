"""Generic leader->follower linear joint mapping (independent of specific leader hardware).

Formula: rad_i = base_rad_i + direction_i * scale_i * radians(deg_i - zero_deg_i)
The input angle is first EMA-smoothed (first frame passes through directly); joints with
missing readings hold their last output.
Calibration: tools/calib_star_map.py generates the JSON via the two-pose method.
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

    def rebase(self, raw_deg: dict[int, float], base_rads: list[float]) -> None:
        """Reset the anchor to the current pose: zero_deg = current leader angle, base_rad = current follower angle.

        Following switches to relative mode (starting from each side's current pose, zero jump
        at startup), immune to anchor staleness / multi-turn count drift. Joints with missing
        readings or non-finite follower positions keep their original anchor unchanged.
        """
        for i, j in enumerate(self._joints):
            deg = raw_deg.get(j["servo"])
            if deg is None or not math.isfinite(base_rads[i]):
                continue
            j["zero_deg"] = deg
            j["base_rad"] = base_rads[i]
            self._smooth[i] = deg
        self._last_out = [j["base_rad"] for j in self._joints]

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
