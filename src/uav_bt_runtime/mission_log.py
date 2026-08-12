"""Structured mission logging without raw flight telemetry."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Protocol, Union

from .clock import Clock


class MissionLogger(Protocol):
    """Minimal structured logger used by runtime components."""

    def emit(self, record_type: str, **fields: Any) -> None:
        """Record one mission event."""

    def close(self) -> None:
        """Release logger resources."""


class NullMissionLogger:
    """Logger used when persistence is not requested."""

    def emit(self, record_type: str, **fields: Any) -> None:
        del record_type, fields

    def close(self) -> None:
        return None


class MemoryMissionLogger:
    """Logger that keeps records for tests and simulation summaries."""

    def __init__(self, clock: Clock) -> None:
        self.clock = clock
        self.records: List[Dict[str, Any]] = []

    def emit(self, record_type: str, **fields: Any) -> None:
        self.records.append(
            {"timestamp": self.clock.time(), "record_type": record_type, **fields}
        )

    def close(self) -> None:
        return None


class JsonlMissionLogger:
    """Append-only JSON Lines logger for one mission executor."""

    def __init__(self, path: Union[str, Path], clock: Clock) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.clock = clock
        self._file = self.path.open("a", encoding="utf-8")

    def emit(self, record_type: str, **fields: Any) -> None:
        record = {"timestamp": self.clock.time(), "record_type": record_type, **fields}
        self._file.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
        self._file.flush()

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()


def make_logger(path: Optional[Union[str, Path]], clock: Clock) -> MissionLogger:
    """Create a JSONL logger when a path is supplied, otherwise a null logger."""

    return JsonlMissionLogger(path, clock) if path is not None else NullMissionLogger()
