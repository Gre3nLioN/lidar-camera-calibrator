"""Semantic, Qt-free commands emitted by the calibration UI."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping


@dataclass(frozen=True)
class CalibrationCommand:
    """A stable command boundary between views and the workspace controller."""

    name: str
    payload: Mapping[str, Any]

    @classmethod
    def make(cls, name: str, **payload: Any) -> "CalibrationCommand":
        return cls(name, dict(payload))


def command(name: str, **payload: Any) -> CalibrationCommand:
    return CalibrationCommand.make(name, **payload)
