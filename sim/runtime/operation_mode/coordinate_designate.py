from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .base import OperationMode


@dataclass
class ModeCoordinateDesignate(OperationMode):
    """Mode 1: 좌표 지정 (Coordinate Designate)."""

    target_coordinate: tuple[float, float, float] | None = None  # (x, y, z)

    def __init__(self, target_coordinate: tuple[float, float, float] | None = None):
        super().__init__(mode_id=1)
        self.target_coordinate = target_coordinate

    def apply(self, *args: Any, **kwargs: Any) -> None:
        # TODO: point sensor/aircraft toward target_coordinate
        pass
