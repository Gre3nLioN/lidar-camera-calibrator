"""Typed, contextual failures for scene-profile configuration and data."""
from __future__ import annotations

from typing import Any


class ProfileError(ValueError):
    """Base error carrying a stable code and machine-readable context."""

    def __init__(
        self,
        code: str,
        path: str,
        message: str,
        *,
        expected: Any | None = None,
        actual: Any | None = None,
        hint: str | None = None,
    ) -> None:
        self.code = str(code)
        self.path = str(path)
        self.message = str(message)
        self.expected = expected
        self.actual = actual
        self.hint = hint
        super().__init__(str(self))

    def __str__(self) -> str:
        detail = f"{self.code} at {self.path}: {self.message}"
        if self.expected is not None:
            detail += f"; expected {self.expected!r}"
        if self.actual is not None:
            detail += f", got {self.actual!r}"
        if self.hint:
            detail += f". {self.hint}"
        return detail


class ConfigurationError(ProfileError):
    """Source adapter configuration is missing, unknown, or inconsistent."""


class SourceDataError(ProfileError):
    """A configured source payload has an invalid value or representation."""


class SynchronizationError(ProfileError):
    """Required camera or IMU data cannot be synchronized to the LiDAR timeline."""


class TransformGraphError(ProfileError):
    """Static semantic transforms do not form one unambiguous tree rooted at IMU."""


class ProfileWriteError(ProfileError):
    """Canonical profile output could not be created atomically."""


class ProfileValidationError(ProfileError):
    """A canonical scene MCAP violates its declared schemas or frame index."""


class ProfileVersionError(ProfileValidationError):
    """A canonical profile version is not supported by this package."""
