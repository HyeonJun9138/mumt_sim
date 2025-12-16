from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class OperationMode:
    """Base class for operation modes (stubs for now)."""

    mode_id: int

    def apply(self, *args: Any, **kwargs: Any) -> None:
        """Run this mode's behaviour (placeholder)."""
        raise NotImplementedError
