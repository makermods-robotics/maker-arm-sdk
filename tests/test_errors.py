from maker_arm import errors


def test_hierarchy():
    for cls in (errors.ConnectError, errors.ParamTimeout, errors.StateError):
        assert issubclass(cls, errors.MakerArmError)


def test_fault_text():
    assert errors.fault_text(0) == "no fault"
    assert "overtemperature" in errors.fault_text(0b000100)
    joined = errors.fault_text(0b000011)
    assert "undervoltage" in joined and "overcurrent" in joined and "|" in joined
    assert "bit5" in errors.fault_text(0b100000) or "not calibrated" in errors.fault_text(0b100000)
    assert errors.fault_text(1 << 6) == "bit6"   # out-of-range bits must also be reported loudly, never an empty string
