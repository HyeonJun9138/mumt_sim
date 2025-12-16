from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .base import OperationMode


@dataclass
class ModeLineSearch(OperationMode):
    """Mode 2: 구간 수색 (Line Search)."""

    line_start: tuple[float, float, float] | None = None
    line_end: tuple[float, float, float] | None = None

    def __init__(
        self,
        line_start: tuple[float, float, float] | None = None,
        line_end: tuple[float, float, float] | None = None,
    ):
        super().__init__(mode_id=2)
        self.line_start = line_start
        self.line_end = line_end

    def apply(self, *args: Any, **kwargs: Any) -> None:
        # TODO: sweep along the line between start/end
        pass
