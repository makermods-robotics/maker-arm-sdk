from maker_arm.transport.socketcan import CAN_EFF_FLAG, _frame_id


def test_frame_id_extended_sets_eff_flag():
    assert _frame_id(0x1900FD07, True) == (0x1900FD07 | CAN_EFF_FLAG)


def test_frame_id_standard_no_flag_and_masked():
    assert _frame_id(7, False) == 7
    assert _frame_id(0x807, False) == 0x007   # 标准帧 11 位掩码
