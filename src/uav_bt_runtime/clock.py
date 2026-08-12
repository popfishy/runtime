"""Clock abstractions for real execution and deterministic simulation."""

from __future__ import annotations

import time
from typing import Protocol


class Clock(Protocol):
    """Time source used by commands, timeouts, and logs."""

    def monotonic(self) -> float:
        """Return monotonic seconds."""

    def time(self) -> float:
        """Return wall-clock seconds."""


class SystemClock:
    """Production clock backed by the Python standard library."""

    def monotonic(self) -> float:
        return time.monotonic()

    def time(self) -> float:
        return time.time()


class ManualClock:
    """Explicitly advanced clock used by tests and joint simulation."""

    def __init__(self, epoch: float = 1_700_000_000.0) -> None:
        self._monotonic = 0.0
        self._epoch = epoch

    def monotonic(self) -> float:
        return self._monotonic

    def time(self) -> float:
        return self._epoch + self._monotonic

    def advance(self, seconds: float) -> None:
        if seconds < 0:
            raise ValueError("manual clock cannot move backwards")
        self._monotonic += seconds
