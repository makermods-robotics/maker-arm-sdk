"""Exception types. Low-frequency ops paths raise exceptions; high-frequency control paths only log."""


class ArmError(Exception):
    pass


class ConnectError(ArmError):
    pass


class ParamTimeout(ArmError):
    pass


class StateError(ArmError):
    pass


class RouteUnverified(StateError):
    """After an id change it is unclear which motor answers on a joint's id (both ids answer)."""


class FaultLatched(ArmError):
    """A command that would clear fault evidence was refused because a fault is latched."""
