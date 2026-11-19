"""The leader interface the teleop loop drives."""

from __future__ import annotations

import threading
from typing import Optional, Protocol, runtime_checkable


class LeaderError(RuntimeError):
    """A leader could not be connected or configured."""


@runtime_checkable
class Leader(Protocol):
    """A source of follower joint targets.

    ``read()`` returns one target per follower joint (rad, follower joint coordinates, already
    inside the follower's limits), or None when this cycle has no trustworthy reading. It must
    not block much longer than one control period; the caller counts consecutive Nones and
    stops following after too many.
    """

    name: str

    def connect(self, cancel: Optional[threading.Event] = None) -> None:
        """Make the leader readable. ``cancel`` (set by Ctrl-C) aborts a connect that energizes
        anything."""
        ...

    def read(self) -> Optional[list[float]]: ...

    def close(self) -> None: ...
