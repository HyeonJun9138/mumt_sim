from __future__ import annotations

from typing import Any

from .auto_tracking import ModeAutoTracking
from .aircraft_fixed import ModeAircraftFixed
from .coordinate_designate import ModeCoordinateDesignate
from .line_search import ModeLineSearch
from .base import OperationMode


def build_operation_mode(mode_id: int, **kwargs: Any) -> OperationMode:
    """Factory: return stub instance for mode 1~4; others raise ValueError."""
    if mode_id == 1:
        return ModeCoordinateDesignate(**kwargs)
    if mode_id == 2:
        return ModeLineSearch(**kwargs)
    if mode_id == 3:
        return ModeAutoTracking(**kwargs)
    if mode_id == 4:
        return ModeAircraftFixed(**kwargs)
    raise ValueError(f"Unsupported operation mode: {mode_id}")
