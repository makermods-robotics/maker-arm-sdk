"""Transport-layer interface: moves bytes only, no protocol knowledge.

Convention: the recv callback must be fast in / fast out (parse + write cache); it must
not do control computation or blocking IO. There is one callback per backend; a component that
needs to observe every frame for a while (a motor's id change) reads ``recv_callback``, installs a
callback that chains to it, and puts it back afterwards.
"""

from abc import ABC, abstractmethod
from typing import Callable, Optional

RecvCallback = Callable[[int, bytes], None]


class CanId(int):
    """Numeric arbitration ID retaining the standard/extended frame distinction."""

    def __new__(cls, value: int, extended: bool = True):
        instance = super().__new__(cls, value)
        instance.extended = extended
        return instance


class CanBackend(ABC):
    # Adapters that cannot absorb a burst of replies set this: the control loop then waits for
    # each motor's reply (up to feedback_wait_timeout seconds) before commanding the next one.
    feedback_paced_control: bool = False
    feedback_wait_timeout: float = 0.015

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def send(self, can_id: int, data: bytes, extended: bool = True) -> None: ...

    _cb: Optional[RecvCallback] = None

    def set_recv_callback(self, cb: Optional[RecvCallback]) -> None:
        """Every received frame goes to ``cb(can_id, data)`` (None: frames are dropped)."""
        self._cb = cb

    @property
    def recv_callback(self) -> Optional[RecvCallback]:
        """The callback installed by set_recv_callback(), or None."""
        return self._cb


_BACKENDS = {
    "socketcan": (".socketcan", "SocketCanBackend"),
    "slcan": (".slcan_serial", "SlcanSerialBackend"),
    "gs_usb": (".gs_usb", "GsUsbBackend"),
    "mock": (".mock", "MockBackend"),
}


def create_backend(name: str, **kwargs) -> CanBackend:
    if name not in _BACKENDS:
        raise ValueError(f"unknown backend: {name!r} (choices: {'/'.join(_BACKENDS)})")
    from importlib import import_module
    module, cls = _BACKENDS[name]
    return getattr(import_module(module, __package__), cls)(**kwargs)
