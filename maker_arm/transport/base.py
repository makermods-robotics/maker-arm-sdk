"""传输层接口：只搬字节，不懂协议。

约定：recv 回调必须快进快出（解析+写缓存），不得做控制计算或阻塞 IO。
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
    raise ValueError(f"未知后端: {name!r}（可选 socketcan/at/mock）")
