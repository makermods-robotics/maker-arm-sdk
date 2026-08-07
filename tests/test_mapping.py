import json
import math

import pytest

from maker_arm.mapping import JointMapper

CFG = {
    "alpha": 1.0,   # 测试用 1.0 = 无平滑
    "joints": [
        {"servo": 0, "zero_deg": 10.0, "base_rad": 0.5, "direction": 1.0, "scale": 2.0},
        {"servo": 3, "zero_deg": 0.0, "base_rad": 0.0, "direction": -1.0, "scale": 1.0},
    ],
}


def make(tmp_path, cfg=CFG):
    f = tmp_path / "map.json"
    f.write_text(json.dumps(cfg))
    return JointMapper.from_json(str(f))


def test_linear_map(tmp_path):
    m = make(tmp_path)
    out = m.map({0: 40.0, 3: -90.0})
    assert out[0] == pytest.approx(0.5 + 2.0 * math.radians(30.0))
    assert out[1] == pytest.approx(-1.0 * math.radians(-90.0))


def test_missing_servo_holds_last(tmp_path):
    m = make(tmp_path)
    first = m.map({0: 40.0, 3: -90.0})
    out = m.map({0: 50.0})            # servo 3 缺读
    assert out[1] == first[1]
    assert out[0] != first[0]


def test_rebase_anchors_to_current_pose(tmp_path):
    m = make(tmp_path)
    m.rebase({0: 100.0, 3: -30.0}, [1.2, -0.8])
    out = m.map({0: 100.0, 3: -30.0})       # 未动 → 输出=当前 follower 姿态，零跳变
    assert out == [pytest.approx(1.2), pytest.approx(-0.8)]
    out = m.map({0: 110.0, 3: -30.0})       # leader 动 10° → follower 相对当前姿态偏移
    assert out[0] == pytest.approx(1.2 + 2.0 * math.radians(10.0))
    assert out[1] == pytest.approx(-0.8)


def test_rebase_keeps_missing_servo_anchor(tmp_path):
    m = make(tmp_path)
    m.rebase({0: 100.0}, [1.2, float("nan")])   # servo3 缺读/位置无效 → 保持原锚
    assert m._joints[0]["zero_deg"] == 100.0
    assert m._joints[1]["zero_deg"] == 0.0      # 原值


def test_ema_smoothing(tmp_path):
    import copy
    cfg = copy.deepcopy(CFG)
    cfg["alpha"] = 0.5
    m = make(tmp_path, cfg)
    m.map({0: 0.0, 3: 0.0})           # 首帧直通
    out = m.map({0: 10.0, 3: 0.0})    # EMA: 0.5*10
    assert out[0] == pytest.approx(0.5 + 2.0 * math.radians(0.5 * 10.0 - 10.0))
