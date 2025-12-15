from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .base import OperationMode


@dataclass
class ModeAutoTracking(OperationMode):
    """Mode 3: 자동 추적 (Auto Tracking)."""

    target_id: int | None = None

    def __init__(self, target_id: int | None = None):
        super().__init__(mode_id=3)
        self.target_id = target_id

    def apply(self, *args: Any, **kwargs: Any) -> None:
        # TODO: track the given target
        pass
