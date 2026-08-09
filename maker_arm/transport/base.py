"""Transport-layer interface: moves bytes only, no protocol knowledge.

Convention: the recv callback must be fast in / fast out (parse + write cache); it must
not do control computation or blocking IO.
"""

from abc import ABC, abstractmethod
from typing import Callable


class CanBackend(ABC):
    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def send(self, can_id: int, data: bytes) -> None: ...

    @abstractmethod
    def set_recv_callback(self, cb: Callable[[int, bytes], None]) -> None: ...


def create_backend(name: str, **kwargs) -> CanBackend:
    if name == "socketcan":
        from .socketcan import SocketCanBackend
        return SocketCanBackend(**kwargs)
    if name == "at":
        from .at_serial import AtSerialBackend
        return AtSerialBackend(**kwargs)
    if name == "mock":
        from .mock import MockBackend
        return MockBackend(**kwargs)
    raise ValueError(f"unknown backend: {name!r} (choices: socketcan/at/mock)")
