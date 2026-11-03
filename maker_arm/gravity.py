"""Maker arm gravity torques for the feed-forward term of its MIT frames.

Per joint: its axis dotted with the moment of the weights of every link it carries. Angles and
torques are the motors' own: raw motor degrees in, motor-sign N*m out, independent of a joint's
``direction`` and ``offset`` in the SDK config. The URDF is not package data, so its origins and
inertials are kept here; tests/unit/test_gravity.py holds them equal to urdf/maker_arm/robot.urdf.
"""

import math
from typing import Sequence

DEFAULT_GRAVITY_SCALE = 1.0  # 1 = the URDF model, 0 = no feed-forward
MAX_GRAVITY_SCALE = 1.2

G = 9.81
RAMP_S = 0.5         # ease-in from zero after torque-on
RAMP_STEP_S = 0.05   # the most one frame may advance it, so a pause never skips it
FRESH_S = 0.15       # a measured pose needs a reply from every joint within this
LIMIT_NM = (0.5, 15.0, 7.5, 2.0, 0.5, 0.5)

JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll")

# The URDF's joint origins under a root rotation, with the wrist-yaw axis flipped: raw motor sign.
ROOT_RPY = (math.pi / 2, 0.0, math.pi)
CHAIN = (  # xyz, rpy, axis; every axis is +-z
    ((0.0, 0.030799999, 0.0), (-1.5707963, 0.0, 0.0), (0.0, 0.0, -1.0)),
    ((0.0, 0.0, 0.0583), (1.5707963, 0.0, 0.0), (0.0, 0.0, -1.0)),
    ((0.19, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0)),
    ((-0.174, 0.076315136, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0)),
    ((-0.084967787, 0.032692166, 0.0), (1.5707963, 0.0, 0.17453293), (0.0, 0.0, -1.0)),
    ((-0.00025000013, 0.0, 0.04695), (0.0, 1.5707963, 0.0), (0.0, 0.0, 1.0)),
)
LINKS = (  # mass kg, centre of mass m: link_002..link_007
    (0.55, (0.00013418072, 0.00023828401, 0.046440916)),
    (0.7, (0.13427204, -0.00029654773, 0.00022721862)),
    (0.9018, (-0.11644464, 0.04526336, 0.0010285217)),
    (0.466, (-0.064496017, 0.039703674, 0.00044218785)),
    (0.40775, (0.000825995, 2.791615e-05, 0.04138047)),
    (0.32675856, (0.00719177884972, -3.46267489324e-05, -0.0706666548507)),
)
JAWS = (  # lumped into link_007 at their closed position: gripper_left, gripper_right
    (0.069790549, (-0.00903758844484, 0.0134956640657, -0.13754321128)),
    (0.06945089, (-0.0085474469455, -0.0134801013117, -0.137688742328)),
)


def _mul(a, b):
    return tuple(
        (r[0] * b[0][0] + r[1] * b[1][0] + r[2] * b[2][0],
         r[0] * b[0][1] + r[1] * b[1][1] + r[2] * b[2][1],
         r[0] * b[0][2] + r[1] * b[1][2] + r[2] * b[2][2]) for r in a)


def _apply(m, v):
    return tuple(r[0] * v[0] + r[1] * v[1] + r[2] * v[2] for r in m)


def _about_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _rpy(r, p, y):
    cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p), math.sin(p)
    return _mul(_about_z(y), ((cp, sp * sr, sp * cr), (0.0, cr, -sr), (-sp, cp * sr, cp * cr)))


def _lumped_links():
    mass, com = LINKS[-1]
    moment = [mass * c for c in com]
    for jaw_mass, jaw_com in JAWS:
        mass += jaw_mass
        moment = [t + jaw_mass * c for t, c in zip(moment, jaw_com)]
    return LINKS[:-1] + ((mass, tuple(t / mass for t in moment)),)


_ROOT = _rpy(*ROOT_RPY)
_FIXED = tuple((xyz, _rpy(*rpy), axis) for xyz, rpy, axis in CHAIN)
_LUMPED = _lumped_links()


def model_applies(joint_names: Sequence[str]) -> bool:
    """The model only describes a config whose first joints are the Maker arm's six."""
    return tuple(joint_names[:len(JOINT_NAMES)]) == JOINT_NAMES


def gravity_nm(q_deg: Sequence[float], scale: float = 1.0) -> list[float]:
    """Per joint (motor sign, N*m): the torque that holds the arm at ``q_deg`` (six raw motor degrees)."""
    if not all(math.isfinite(q) for q in q_deg):
        return [math.nan] * len(JOINT_NAMES)
    rot, origin = _ROOT, (0.0, 0.0, 0.0)
    frames = []
    for (xyz, fixed, axis), (_, com), q in zip(_FIXED, _LUMPED, q_deg):
        origin = tuple(o + d for o, d in zip(origin, _apply(rot, xyz)))
        rot = _mul(rot, fixed)
        world_axis = _apply(rot, axis)
        rot = _mul(rot, _about_z(axis[2] * math.radians(q)))
        frames.append((world_axis, origin, tuple(o + d for o, d in zip(origin, _apply(rot, com)))))
    # Weights are vertical: a . ((c - p) x (0, 0, m g)) = m g (a_x (c_y - p_y) - a_y (c_x - p_x)).
    out = [0.0] * len(frames)
    mass = moment_x = moment_y = 0.0
    for i in reversed(range(len(frames))):
        axis, pivot, com = frames[i]
        m = _LUMPED[i][0]
        mass += m
        moment_x += m * com[0]
        moment_y += m * com[1]
        out[i] = scale * G * (axis[0] * (moment_y - mass * pivot[1]) - axis[1] * (moment_x - mass * pivot[0]))
    return out


class GravityFeedforward:
    """The six joints' torques for one session of torque-on, with its ease-in and guards."""

    def __init__(self, scale: float = DEFAULT_GRAVITY_SCALE):
        if not 0.0 <= scale <= MAX_GRAVITY_SCALE:
            raise ValueError(f"gravity_scale must be in [0, {MAX_GRAVITY_SCALE}], got {scale}")
        self.scale = scale
        self.level = 0.0
        self._last = None

    def torques(self, measured_deg: Sequence[float], ages: Sequence[float], goal_deg: Sequence[float],
                now: float) -> list[float]:
        """Torques for the frames about to go out, one step further into the ease-in: at the measured
        pose when every joint replied within FRESH_S, else at the goal. Non-finite values become 0."""
        if self._last is not None:
            self.level = min(1.0, self.level + min(RAMP_STEP_S, max(0.0, now - self._last)) / RAMP_S)
        self._last = now
        fresh = all(math.isfinite(q) and age <= FRESH_S for q, age in zip(measured_deg, ages))
        tau = gravity_nm(measured_deg if fresh else goal_deg, self.scale)
        return [max(-limit, min(limit, t * self.level)) if math.isfinite(t) else 0.0
                for t, limit in zip(tau, LIMIT_NM)]
