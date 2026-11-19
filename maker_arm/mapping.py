"""Leader servo degrees -> follower joint radians, from a mapping JSON (``profiles/star_to_*.json``).

Leader and follower are zeroed in the same pose, so per joint
``follower_deg = unwrap(raw_deg) * direction + offset_deg``, clipped to ``range_deg``.
``direction`` carries a scale as well as the sign (leader and follower travel differ). The unwrap
removes whole servo turns, centring the window on the follower range expressed in leader degrees
((range - offset) / direction). ``"magnitude": true`` maps the distance from zero instead of the
signed reading, for a trigger grip that some builds turn one way and some the other.

JSON: ``{"joints": [{"name", "servo", "direction", "range_deg": [lo, hi], "offset_deg"?, "magnitude"?}]}``
"""

import json
import math


class JointMapper:
    def __init__(self, joints: list[dict]):
        for j in joints:
            if not j["direction"] or not math.isfinite(j["direction"]):
                raise ValueError(f"servo {j['servo']}: direction must be finite and non-zero")
            lo, hi = j["range_deg"]
            if not lo < hi:
                raise ValueError(f"servo {j['servo']}: range_deg must satisfy lo < hi")
        ids = [j["servo"] for j in joints]
        if len(set(ids)) != len(ids):
            raise ValueError(f"servo ids must be unique: {ids}")
        self._joints = joints
        self._last_out: list[float] = [math.nan] * len(joints)

    @classmethod
    def from_json(cls, path) -> "JointMapper":
        with open(path) as f:
            raw = json.load(f)
        return cls(raw["joints"])

    @property
    def servo_ids(self) -> list[int]:
        return [j["servo"] for j in self._joints]

    @property
    def names(self) -> list[str]:
        return [j.get("name", f"servo {j['servo']}") for j in self._joints]

    @staticmethod
    def unwrap(value: float, lo: float, hi: float) -> float:
        center = (lo + hi) / 2.0
        return value - round((value - center) / 360.0) * 360.0

    def map_joint(self, index: int, deg: float) -> float:
        """One joint's follower target (rad) for a raw leader reading (deg)."""
        j = self._joints[index]
        lo, hi = j["range_deg"]
        direction = j["direction"]
        offset = j.get("offset_deg", 0.0)
        if j.get("magnitude"):
            position = abs(self.unwrap(deg, -180.0, 180.0)) * direction + offset
        else:
            a, b = (lo - offset) / direction, (hi - offset) / direction
            position = self.unwrap(deg, min(a, b), max(a, b)) * direction + offset
        return math.radians(max(lo, min(hi, position)))

    def map(self, raw_deg: dict[int, float]) -> list[float]:
        """Leader raw degrees by servo ID -> follower targets in radians.

        A joint without a reading keeps its last output (NaN before its first reading, which
        Arm.set_joint_targets rejects).
        """
        out = list(self._last_out)
        for i, j in enumerate(self._joints):
            deg = raw_deg.get(j["servo"])
            if deg is None or not math.isfinite(deg):
                continue
            out[i] = self.map_joint(i, deg)
        self._last_out = out
        return out

    def reset(self) -> None:
        self._last_out = [math.nan] * len(self._joints)
