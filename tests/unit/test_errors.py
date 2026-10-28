from maker_arm import errors


def test_hierarchy():
    for cls in (errors.ConnectError, errors.ParamTimeout, errors.StateError, errors.FaultLatched):
        assert issubclass(cls, errors.MakerArmError)
