"""This repo's arm chain as tests/unit/gravity_fixtures.json describes it (URDF, signs, base turn,
TCP, soft limits) and the reference holding torques it must reproduce."""

import importlib
import json
import math
from pathlib import Path

import numpy as np

from conftest import sdk

ROOT = Path(__file__).resolve().parents[2]
FIXTURES = json.loads((Path(__file__).resolve().parent / "gravity_fixtures.json").read_text())
SPEC = FIXTURES["chain"]
CASES = FIXTURES["cases"]
LIMITS = np.radians(np.array(SPEC["limits_deg"], dtype=float)).T  # (lower, upper)


def load(module: str):
    """A submodule of the package under test, e.g. load("kinematics")."""
    return importlib.import_module(f"{sdk.__name__}.{module}")


def repo_chain(tcp: bool = True):
    kinematics = load("kinematics")
    return kinematics.Chain.from_urdf(
        ROOT / SPEC["urdf"],
        sign=SPEC["sign"],
        offset=np.radians(SPEC["offset_deg"]),
        base_rpy=[math.radians(v) for v in SPEC["base_rpy_deg"]],
        tcp_xyz=SPEC["tcp_xyz"] if tcp else (0, 0, 0),
        tcp_rpy=[math.radians(v) for v in SPEC["tcp_rpy_deg"]] if tcp else (0, 0, 0),
    )
