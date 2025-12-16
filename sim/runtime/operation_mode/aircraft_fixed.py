from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from sim.world.dem import ray_intersect_dem
from .base import OperationContext, OperationMode, OperationResult


@dataclass
class ModeAircraftFixed(OperationMode):
    """Mode 4: keep gimbal fixed relative to aircraft orientation."""

    mode_id: int = 4

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
        aircraft_fixed = filming_prop.get("aircraftFixed") or {}
        pitch_deg = float(aircraft_fixed.get("gimbalPitch", 0.0))
        yaw_offset_deg = float(aircraft_fixed.get("gimbalYaw", 0.0))
        yaw_deg = uav.s.yaw + yaw_offset_deg
        yaw_rad = math.radians(yaw_deg)
        pitch_rad = math.radians(pitch_deg)
        cp = math.cos(pitch_rad)
        dir_vec = np.array(
            [cp * math.cos(yaw_rad), -cp * math.sin(yaw_rad), math.sin(pitch_rad)],
            dtype=float,
        )
        origin = np.array([uav.s.x, uav.s.y, uav.s.z], dtype=float)
        hit = ray_intersect_dem(origin, dir_vec, ctx.dem)
        if hit is not None:
            target = tuple(hit.tolist())
        else:
            target = tuple((origin + dir_vec * 2000.0).tolist())
        return OperationResult(target=target, state=None)
