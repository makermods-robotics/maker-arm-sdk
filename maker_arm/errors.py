"""Exception types and fault code translation. Low-frequency ops paths raise exceptions; high-frequency control paths only log."""


class MakerArmError(Exception):
    pass


class ConnectError(MakerArmError):
    pass


class ParamTimeout(MakerArmError):
    pass


class StateError(MakerArmError):
    pass


# The 6-bit fault code at feedback frame ID Bit16~21, per the RobStride common definition.
# TODO-hardware verification: cross-check the bit definitions once during real-hardware bring-up by deliberately triggering a fault (e.g. undervoltage).
FAULT_NAMES = {
    0: "undervoltage",
    1: "overcurrent",
    2: "overtemperature",
    3: "magnetic encoder fault",
    4: "hall encoder fault",
    5: "encoder not calibrated",
}


def fault_text(bits: int) -> str:
    if not bits:
        return "no fault"
    names = [FAULT_NAMES.get(i, f"bit{i}") for i in range(max(6, bits.bit_length())) if bits & (1 << i)]
    return "|".join(names)
