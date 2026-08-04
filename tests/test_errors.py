from maker_arm import errors


def test_hierarchy():
    for cls in (errors.ConnectError, errors.ParamTimeout, errors.StateError):
        assert issubclass(cls, errors.MakerArmError)


def test_fault_text():
    assert errors.fault_text(0) == "无故障"
    assert "过温" in errors.fault_text(0b000100)
    joined = errors.fault_text(0b000011)
    assert "欠压" in joined and "过流" in joined and "|" in joined
    assert "bit5" in errors.fault_text(0b100000) or "未标定" in errors.fault_text(0b100000)
