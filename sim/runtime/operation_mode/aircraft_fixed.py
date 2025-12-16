from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .base import OperationMode


@dataclass
class ModeAircraftFixed(OperationMode):
    """Mode 4: 기체 고정 (Aircraft Fixed)."""

    fixed_offset: tuple[float, float, float] | None = None

    def __init__(self, fixed_offset: tuple[float, float, float] | None = None):
        super().__init__(mode_id=4)
        self.fixed_offset = fixed_offset

    def apply(self, *args: Any, **kwargs: Any) -> None:
        # TODO: keep camera fixed relative to aircraft body/orientation
        pass
