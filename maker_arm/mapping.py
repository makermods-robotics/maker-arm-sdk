"""Star Arm 102 leader -> Maker follower joint mapping, identical to LeRobot's
RebotArm102LeaderMakerConfig (makermods-robotics/lerobot@5ce2fe41a).

Both arms are zeroed in the same pose (folded against the base, gripper fully closed), so the
mapping has no offset: ``follower_deg = unwrap(raw_deg) * direction``, clipped to the joint's
range. ``direction`` carries a scale as well as the sign. The unwrap removes whole servo turns,
centring the window on the follower range expressed in leader degrees (range / direction).
"""

import json
import math


class JointMapper:
    def __init__(self, joints: list[dict]):
        for j in joints:
            if not j["direction"]:
                raise ValueError(f"servo {j['servo']}: direction must be non-zero")
            lo, hi = j["range_deg"]
            if not lo < hi:
                raise ValueError(f"servo {j['servo']}: range_deg must satisfy lo < hi")
        self._joints = joints
        self._last_out: list[float] = [math.nan] * len(joints)

    @classmethod
    def from_json(cls, path: str) -> "JointMapper":
        with open(path) as f:
            raw = json.load(f)
        return cls(raw["joints"])

    @property
    def servo_ids(self) -> list[int]:
        return [j["servo"] for j in self._joints]

    @staticmethod
    def unwrap(value: float, lo: float, hi: float) -> float:
        center = (lo + hi) / 2.0
        return value - round((value - center) / 360.0) * 360.0

    def map(self, raw_deg: dict[int, float]) -> list[float]:
        """Leader raw degrees by servo ID -> follower targets in radians.

        A joint without a reading keeps its last output (NaN before its first reading, which
        Arm.set_joint_targets rejects).
        """
        out = list(self._last_out)
        for i, j in enumerate(self._joints):
            deg = raw_deg.get(j["servo"])
            if deg is None:
                continue
            lo, hi = j["range_deg"]
            direction = j["direction"]
            position = self.unwrap(deg, lo / direction, hi / direction) * direction
            out[i] = math.radians(max(lo, min(hi, position)))
        self._last_out = out
        return out
