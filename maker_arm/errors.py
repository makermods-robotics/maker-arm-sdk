"""Exception types. Low-frequency ops paths raise exceptions; high-frequency control paths only log."""


class MakerArmError(Exception):
    pass


class ConnectError(MakerArmError):
    pass


class ParamTimeout(MakerArmError):
    pass


class StateError(MakerArmError):
    pass


class FaultLatched(MakerArmError):
    """A command that would clear fault evidence was refused because a fault is latched."""
