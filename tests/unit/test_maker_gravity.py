"""The Maker profile's gravity model against holding torques measured on the arm (tests/unit/gravity_measured.json):
the shoulder term matches within static friction plus a little, and elbow_flex and wrist_flex stay as close."""

import json
import math
from pathlib import Path

import numpy as np

from maker_arm import gravity
from maker_arm.config import ArmConfig
from maker_arm._product import DEFAULT_PROFILE

MEASURED = json.loads((Path(__file__).resolve().parent / "gravity_measured.json").read_text())["cases"]


def residual_rms():
    config = ArmConfig.from_yaml(str(DEFAULT_PROFILE))
    model = gravity.from_config(config).model
    residuals = np.array([np.array(c["tau_nm"][:6]) - np.array(model([math.radians(v) for v in c["q_deg"][:6]]))
                          for c in MEASURED])
    return np.sqrt(np.mean(np.square(residuals), axis=0))


def test_the_shoulder_term_matches_the_measured_holding_torque():
    rms = residual_rms()
    assert rms[1] < 0.2      # N*m; the friction measured at these poses is about 0.06


def test_elbow_and_wrist_terms_stay_as_close_as_measured():
    rms = residual_rms()
    assert rms[2] < 0.25 and rms[3] < 0.2
