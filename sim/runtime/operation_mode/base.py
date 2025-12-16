from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, TYPE_CHECKING

if TYPE_CHECKING:
    from sim.world.dem import DEM

TargetCoord = tuple[float, float, float]


@dataclass
class OperationContext:
    """Shared dependencies passed into each mode."""

    dem: DEM
    default_target_fn: Callable[[Any], TargetCoord]


@dataclass
class OperationResult:
    """Result of applying an operation mode for one simulation tick."""

    target: TargetCoord | None
    state: Any = None
    debug: Any = None
    reset_debug: bool = False


@dataclass
class OperationMode:
    """Base class for operation modes (stubs for now)."""

    mode_id: int

    def apply(
        self,
        *,
        uav: Any,
        filming_prop: dict,
        ctx: OperationContext,
        dt: float,
        current_wp_id: int | None,
        prev_state: Any,
    ) -> OperationResult:
        """Run this mode's behaviour and return the next target/state."""
        raise NotImplementedError
