"""异常类型与故障码翻译。低频运维路径抛异常；高频控制路径只记日志。"""


class MakerArmError(Exception):
    pass


class ConnectError(MakerArmError):
    pass


class ParamTimeout(MakerArmError):
    pass


class StateError(MakerArmError):
    pass


# 反馈帧 ID Bit16~21 的 6 位故障码，按 RobStride 通用定义。
# TODO-硬件验证：真机 bring-up 时人为触发（如欠压）对照一次位定义。
FAULT_NAMES = {
    0: "欠压",
    1: "过流",
    2: "过温",
    3: "磁编码故障",
    4: "HALL 编码故障",
    5: "编码器未标定",
}


def fault_text(bits: int) -> str:
    if not bits:
        return "无故障"
    names = [FAULT_NAMES.get(i, f"bit{i}") for i in range(6) if bits & (1 << i)]
    return "|".join(names)
