import pytest

from conftest import feedback_frame
from maker_arm import protocol as p
from maker_arm.arm import Arm
from maker_arm.config import ArmConfig, JointConfig
from maker_arm.transport.mock import MockBackend


def test_motor_params_table():
    assert p.MOTOR_PARAMS["RS00"].t_max == 14.0 and p.MOTOR_PARAMS["RS00"].v_max == 33.0
    assert p.MOTOR_PARAMS["RS02"].t_max == 17.0 and p.MOTOR_PARAMS["RS02"].v_max == 44.0
    assert p.RS02.p_max == 12.57  # P 映射同 RS00（新固件）


def test_encode_mit_rs02_golden():
    # τ=8.5 Nm → RS02: (8.5+17)*65535/34 = 49151 = 0xBFFF，编进 ID Bit23~8
    cid, data = p.encode_mit(2, 0.0, 0.0, 10.0, 1.0, 8.5, params=p.RS02)
    assert cid == 0x01BFFF02
    assert data == bytes.fromhex("80008000051F3333")  # pos/vel 零点与 RS00 相同
    # v=22 rad/s → RS02 编码 0xBFFF；RS00 编码不同（54612）——证明查表生效
    _, d02 = p.encode_mit(2, 0.0, 22.0, 0.0, 0.0, 0.0, params=p.RS02)
    assert d02[2:4] == bytes.fromhex("BFFF")
    _, d00 = p.encode_mit(2, 0.0, 22.0, 0.0, 0.0, 0.0)
    assert d00[2:4] != d02[2:4]


def test_parse_feedback_rs02():
    cid, data = feedback_frame(3, vel=22.0, tau=8.5, params=p.RS02)
    fb = p.parse_frame(cid, data, params=p.RS02)
    assert fb.velocity == pytest.approx(22.0, abs=2e-3)
    assert fb.torque == pytest.approx(8.5, abs=1e-3)
    # 用错表解码会差一截（22*88/66≠22）——这就是型号选错的经典症状
    fb_wrong = p.parse_frame(cid, data)
    assert fb_wrong.velocity != pytest.approx(22.0, abs=0.5)


def mixed_cfg():
    return ArmConfig(joints=[
        JointConfig(motor_id=1, direction=1, offset=0.0, lo=-3.0, hi=3.0, kp=60, kd=2.0),
        JointConfig(motor_id=2, direction=1, offset=0.0, lo=-3.0, hi=3.0, kp=60, kd=2.0, model="RS02"),
    ])


def test_mixed_arm_decodes_per_joint():
    be = MockBackend()

    def responder(cid, data):
        mid = cid & 0xFF
        if mid == 1:
            return [feedback_frame(1, vel=22.0, tau=7.0)]
        if mid == 2:
            return [feedback_frame(2, vel=22.0, tau=7.0, params=p.RS02)]
    be.responder = responder
    arm = Arm(mixed_cfg(), be)
    arm.connect(timeout=1.0)
    v, t = arm.get_joint_velocities(), arm.get_joint_torques()
    assert v[0] == pytest.approx(22.0, abs=2e-3) and v[1] == pytest.approx(22.0, abs=2e-3)
    assert t[0] == pytest.approx(7.0, abs=2e-3) and t[1] == pytest.approx(7.0, abs=2e-3)


def test_mixed_arm_encodes_per_joint():
    be = MockBackend()
    be.responder = lambda cid, data: [feedback_frame(cid & 0xFF)] if (cid & 0xFF) in (1, 2) else None
    arm = Arm(mixed_cfg(), be)
    arm.connect(timeout=1.0)
    arm.motors[1].send_mit(0.0, 22.0, 0.0, 0.0, 8.5)   # J2=RS02
    cid, data = be.sent[-1]
    assert (cid >> 8) & 0xFFFF == 0xBFFF                # τ 用 ±17 表
    assert data[2:4] == bytes.fromhex("BFFF")           # v 用 ±44 表


def test_config_model_field(tmp_path):
    import yaml
    good = {"joints": [
        {"motor_id": 1, "direction": 1, "offset": 0.0, "lo": -3.0, "hi": 3.0, "kp": 60, "kd": 2.0},
        {"motor_id": 2, "direction": 1, "offset": 0.0, "lo": -3.0, "hi": 3.0, "kp": 60, "kd": 2.0, "model": "RS02"},
    ]}
    f = tmp_path / "a.yaml"
    f.write_text(yaml.safe_dump(good))
    cfg = ArmConfig.from_yaml(str(f))
    assert cfg.joints[0].model == "RS00"    # 缺省 → RS00，老 YAML 兼容
    assert cfg.joints[1].model == "RS02"
    good["joints"][1]["model"] = "RS99"
    f.write_text(yaml.safe_dump(good))
    with pytest.raises(ValueError, match="型号"):
        ArmConfig.from_yaml(str(f))


def test_maker_arm_config_loads():
    cfg = ArmConfig.from_yaml("configs/maker_arm.yaml")
    assert cfg.n_joints == 7
    assert [j.model for j in cfg.joints] == ["RS00", "RS02", "RS02", "RS00", "RS00", "RS00", "RS00"]
    assert cfg.joints[6].motor_id == 7
