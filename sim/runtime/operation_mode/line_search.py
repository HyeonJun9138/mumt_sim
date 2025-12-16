from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from .base import OperationContext, OperationMode, OperationResult, TargetCoord


@dataclass
class ModeLineSearch(OperationMode):
    """Mode 2: line search along supplied coordinates."""

    mode_id: int = 2

    def _convert_points(self, line_search: dict, dem) -> list[TargetCoord]:
        coords = line_search.get("coordinateList") or []
        pts: list[TargetCoord] = []
        for c in coords:
            lon = c.get("longitude")
            lat = c.get("latitude")
            alt = c.get("altitude", 0.0)
            if lon is None or lat is None:
                continue
            try:
                x, y = dem.lonlat_to_env(lon, lat)
                z = dem.get_height(x, y) if alt == 0 else float(alt)
                pts.append((float(x), float(y), float(z)))
            except Exception:
                continue
        return pts

    def _build_segments(self, pts: list[TargetCoord]) -> list[tuple[TargetCoord, TargetCoord]]:
        return [(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]

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
        line_search = filming_prop.get("lineSearch") or {}
        state = prev_state
        debug_points = None

        if state is None or state.get("wp_id") != current_wp_id or state.get("filming_id") != id(filming_prop):
            pts = self._convert_points(line_search, ctx.dem)
            segs = self._build_segments(pts)
            if not segs:
                return OperationResult(target=ctx.default_target_fn(uav), state=None, debug=None, reset_debug=True)
            state = {
                "segments": segs,
                "seg_idx": 0,
                "seg_t": 0.0,
                "speed": float(line_search.get("searchSpeed", 0.0)),
                "wp_id": current_wp_id,
                "filming_id": id(filming_prop),
            }
            debug_points = pts

        segs = state["segments"]
        if not segs:
            return OperationResult(target=ctx.default_target_fn(uav), state=None, debug=None, reset_debug=True)

        seg_idx = state.get("seg_idx", 0)
        seg_t = state.get("seg_t", 0.0)
        speed = max(0.0, float(state.get("speed", 0.0)))
        dist_left = speed * max(0.0, float(dt or 0.0))

        while dist_left > 0.0 and seg_idx < len(segs):
            p0, p1 = segs[seg_idx]
            seg_len = math.dist(p0, p1)
            if seg_len < 1e-6:
                seg_idx += 1
                seg_t = 0.0
                continue
            advance_t = dist_left / seg_len
            new_t = seg_t + advance_t
            if new_t >= 1.0:
                dist_left = (new_t - 1.0) * seg_len
                seg_idx += 1
                seg_t = 0.0
            else:
                seg_t = new_t
                dist_left = 0.0

        if seg_idx >= len(segs):
            seg_idx = len(segs) - 1
            seg_t = 1.0

        p0, p1 = segs[seg_idx]
        target = (
            p0[0] + (p1[0] - p0[0]) * seg_t,
            p0[1] + (p1[1] - p0[1]) * seg_t,
            p0[2] + (p1[2] - p0[2]) * seg_t,
        )

        state["seg_idx"] = seg_idx
        state["seg_t"] = seg_t

        return OperationResult(target=target, state=state, debug=debug_points)
