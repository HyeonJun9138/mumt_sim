"""Standoff-sweep mission and controller (waypoints + scan line gimbal)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import numpy as np

from sim.config import clamp, wrap_deg


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    name: str


@dataclass
class Mission:
    waypoints: List[Waypoint]
    current_idx: int = 0
    active: bool = False
    completed: bool = False


@dataclass(frozen=True)
class MissionPackage:
    waypoint: Tuple[float, float, float]
    scan_start: Tuple[float, float]
    scan_end: Tuple[float, float]
    yaw_speed_dps: float


# (waypoint, scan_start, scan_end)
STANDOFF_MISSION_SPECS: Sequence[Tuple[Tuple[float, float, float], Tuple[float, float] | None, Tuple[float, float] | None]] = [
    ((20399.7, 14617.7, 610.0), (19995.0, 13587.2), (19345.3, 14280.3)),
    ((20311.8, 14535.3, 610.0), (19907.1, 13504.8), (19257.4, 14197.9)),
    ((20223.9, 14452.9, 610.0), (19819.2, 13422.4), (19169.5, 14115.5)),
    ((20136.0, 14370.5, 610.0), (19731.3, 13340.0), (19081.6, 14033.0)),
    ((20048.1, 14288.0, 610.0), (19643.4, 13257.6), (18993.7, 13950.6)),
    ((19960.2, 14205.6, 610.0), (19555.5, 13175.2), (18905.8, 13868.2)),
    ((19872.3, 14123.2, 610.0), (19467.6, 13092.7), (18817.9, 13785.8)),
    ((19784.4, 14040.8, 610.0), (19379.7, 13010.3), (18729.9, 13703.4)),
    ((19696.4, 13958.4, 610.0), (19291.8, 12927.9), (18642.0, 13621.0)),
    ((19608.5, 13876.0, 610.0), (19203.9, 12845.5), (18554.1, 13538.6)),
    ((19520.6, 13793.6, 610.0), (19116.0, 12763.1), (18466.2, 13456.2)),
    ((19432.7, 13711.2, 610.0), (19028.1, 12680.7), (18378.3, 13373.7)),
    ((19344.8, 13628.7, 610.0), (18940.2, 12598.3), (18290.4, 13291.3)),
    ((19256.9, 13546.3, 610.0), (18852.2, 12515.9), (18202.5, 13208.9)),
    ((19169.0, 13463.9, 610.0), (18764.3, 12433.4), (18114.6, 13126.5)),
    ((19081.1, 13381.5, 610.0), (18676.4, 12351.0), (18026.7, 13044.1)),
    ((18993.2, 13299.1, 610.0), (18588.5, 12268.6), (17938.8, 12961.7)),
    ((18905.3, 13216.7, 610.0), (18500.6, 12186.2), (17850.9, 12879.3)),
    ((18817.4, 13134.3, 610.0), (18412.7, 12103.8), (17763.0, 12796.9)),
    ((18729.5, 13051.8, 610.0), (18324.8, 12021.4), (17675.1, 12714.4)),

]

DEFAULT_YAW_SPEED = 140.0
TARGET_SPEED = 60.0
ARRIVAL_DIST_M = 50.0
ENTRY_BLEND_DIST_M = 350.0
REJOIN_PERP_M = 400.0
REJOIN_BACK_M = 200.0


def _angle_diff_deg(a: float, b: float) -> float:
    diff = (a - b + 180.0) % 360.0 - 180.0
    return diff


def _build_packages(
    specs: Sequence[Tuple[Tuple[float, float, float], Tuple[float, float] | None, Tuple[float, float] | None]]
    | None = None,
    yaw_speed_dps: float = DEFAULT_YAW_SPEED,
) -> List[MissionPackage]:
    plan: List[MissionPackage] = []
    specs = specs if specs is not None else STANDOFF_MISSION_SPECS
    for idx, (wp, scan_start, scan_end) in enumerate(specs):
        if scan_start is None or scan_end is None:
            scan_start = scan_end = (wp[0], wp[1])
        elif (idx + 1) % 2 == 0:
            # Flip every other leg to alternate scan direction (matches original script)
            scan_start, scan_end = scan_end, scan_start
        plan.append(
            MissionPackage(
                waypoint=wp,
                scan_start=scan_start,
                scan_end=scan_end,
                yaw_speed_dps=yaw_speed_dps,
            )
        )
    return plan


def _make_waypoints_from_packages(packages: Iterable[MissionPackage]) -> List[Waypoint]:
    wps: List[Waypoint] = []
    for idx, pkg in enumerate(packages):
        x, y, z = pkg.waypoint
        wps.append(Waypoint(x=x, y=y, z=z, name=f"std{idx + 1}"))
    return wps


def build_standoff_missions(
    waypoint_specs: Sequence[Tuple[Tuple[float, float, float], Tuple[float, float] | None, Tuple[float, float] | None]]
    | None = None,
) -> List[Mission]:
    packages = _build_packages(waypoint_specs)
    if not packages:
        return []
    wps = _make_waypoints_from_packages(packages)
    return [Mission(waypoints=wps, current_idx=0, active=True, completed=False)]


class StandoffController:
    """
    Autopilot-like controller for the standoff sweep.
    Moves the aircraft through fixed waypoints and slews the camera between scan_start/end.
    """

    def __init__(
        self,
        uav,
        dem,
        packages: Sequence[MissionPackage] | None = None,
        target_speed: float = TARGET_SPEED,
    ):
        self.uav = uav
        self.dem = dem
        self.mission_plan: List[MissionPackage] = list(_build_packages() if packages is None else packages)
        self.target_speed = target_speed

        self.KP_YAW = 1.2
        self.KP_PITCH = 0.5
        self.KP_ROLL = 1.8
        self.KP_THROTTLE = 0.2

        self.mission_index = 0
        self.mission_done = False
        self.heading_override_deg: float | None = None
        self.entry_heading_deg: float | None = None
        self.rejoin_points: List[np.ndarray] = []
        self.rejoin_points_debug: List[np.ndarray] = []

        self.scan_timer = 0.0
        self.scan_duration = 0.0
        self.current_start = None
        self.current_end = None
        self.current_gimbal_target = None

        self._start_scan(0)

    def resume_from(self, mission_index: int, scan_progress: float = 0.0):
        """Resume standoff mission at the given leg with optional scan progress (0-1)."""
        if not self.mission_plan:
            return
        idx = max(0, min(mission_index, len(self.mission_plan) - 1))
        self.mission_done = False
        self.heading_override_deg = None
        self.mission_index = idx
        # Precompute entry heading toward the outbound leg from this waypoint.
        self.entry_heading_deg = None
        self.rejoin_points = []
        self.rejoin_points_debug = []
        if idx + 1 < len(self.mission_plan):
            curr_wp = self.mission_plan[idx].waypoint
            next_wp = self.mission_plan[idx + 1].waypoint
            dx_leg = next_wp[0] - curr_wp[0]
            dy_leg = next_wp[1] - curr_wp[1]
            if abs(dx_leg) + abs(dy_leg) > 1e-3:
                self.entry_heading_deg = wrap_deg(math.degrees(math.atan2(-dy_leg, dx_leg)))
                # Build helper points: A (perp 400m at 610m alt), B (midpoint shifted back 200m).
                leg_len = math.hypot(dx_leg, dy_leg)
                ux, uy = dx_leg / leg_len, dy_leg / leg_len
                # Choose side based on UAV position relative to leg (left/right).
                side = 1.0
                if hasattr(self.uav, "s"):
                    vx = self.uav.s.x - curr_wp[0]
                    vy = self.uav.s.y - curr_wp[1]
                    cross = ux * vy - uy * vx
                    side = 1.0 if cross >= 0.0 else -1.0
                px, py = -uy * side, ux * side  # perpendicular with chosen side
                ax = curr_wp[0] + px * REJOIN_PERP_M
                ay = curr_wp[1] + py * REJOIN_PERP_M
                az = 610.0
                midx = 0.5 * (ax + curr_wp[0])
                midy = 0.5 * (ay + curr_wp[1])
                bx = midx - ux * REJOIN_BACK_M
                by = midy - uy * REJOIN_BACK_M
                bz = 610.0
                # Visit A first (perpendicular offset), then B (midpoint pulled back).
                self.rejoin_points = [
                    np.array([ax, ay, az], dtype=float),
                    np.array([bx, by, bz], dtype=float),
                ]
                self.rejoin_points_debug = list(self.rejoin_points)
        self._start_scan(idx)

        scan_progress = clamp(scan_progress, 0.0, 1.0)
        if self.scan_duration > 0.0 and self.current_start is not None and self.current_end is not None:
            self.scan_timer = scan_progress * self.scan_duration
            self.current_gimbal_target = (
                self.current_start * (1.0 - scan_progress) + self.current_end * scan_progress
            )

    def _ground_point(self, xy: Tuple[float, float]) -> np.ndarray:
        x, y = xy
        z = self.dem.get_height(x, y)
        return np.array([x, y, z], dtype=float)

    def _compute_scan_duration(self, start: np.ndarray, end: np.ndarray, yaw_speed_dps: float) -> float:
        uav_pos = np.array([self.uav.s.x, self.uav.s.y, self.uav.s.z], dtype=float)
        v_start = start - uav_pos
        v_end = end - uav_pos
        if np.linalg.norm(v_start) < 1e-6 or np.linalg.norm(v_end) < 1e-6:
            return 1.0
        v_start /= np.linalg.norm(v_start)
        v_end /= np.linalg.norm(v_end)
        dot = float(np.clip(np.dot(v_start, v_end), -1.0, 1.0))
        angle_deg = math.degrees(math.acos(dot))
        if yaw_speed_dps <= 0.0:
            return 1.0
        return max(0.5, angle_deg / yaw_speed_dps)

    def _start_scan(self, index: int):
        if index >= len(self.mission_plan):
            self.current_gimbal_target = None
            self.current_start = None
            self.current_end = None
            self.scan_duration = 0.0
            self.scan_timer = 0.0
            return
        mission = self.mission_plan[index]
        self.current_start = self._ground_point(mission.scan_start)
        self.current_end = self._ground_point(mission.scan_end)
        self.scan_timer = 0.0
        self.scan_duration = self._compute_scan_duration(self.current_start, self.current_end, mission.yaw_speed_dps)
        self.current_gimbal_target = np.copy(self.current_start)
        # Visualization helpers for rejoin
        self.rejoin_points_debug = list(self.rejoin_points)

    def _advance_leg(self):
        self.mission_index += 1
        self.heading_override_deg = None
        self.entry_heading_deg = None
        self.rejoin_points = []
        if self.mission_index < len(self.mission_plan):
            self._start_scan(self.mission_index)
        else:
            self.mission_done = True
            self.current_gimbal_target = None
            self.scan_duration = 0.0

    def _update_movement(self, dt: float):
        if self.mission_index >= len(self.mission_plan):
            if not self.mission_done:
                print("[standoff] all waypoints reached.")
                self.mission_done = True
            # gentle slowdown / level out
            self.uav.cmd_throttle = -0.3
            self.uav.cmd_yaw_rate = 0.0
            self.uav.cmd_pitch_rate = -self.uav.s.pitch * 1.0
            self.uav.cmd_roll_rate = -self.uav.s.roll * 1.5
            return

        if self.rejoin_points:
            tx, ty, tz = self.rejoin_points[0]
            arrival_tol = max(20.0, ARRIVAL_DIST_M * 0.6)
        else:
            tx, ty, tz = self.mission_plan[self.mission_index].waypoint
            arrival_tol = ARRIVAL_DIST_M
        dx = tx - self.uav.s.x
        dy = ty - self.uav.s.y
        dz = tz - self.uav.s.z
        dist_2d = math.hypot(dx, dy)

        if dist_2d < arrival_tol:
            if self.rejoin_points:
                self.rejoin_points.pop(0)
            else:
                print(f"[standoff] waypoint {self.mission_index + 1}/{len(self.mission_plan)} reached.")
                self._advance_leg()

        error_z = dz
        target_pitch_rate = clamp(
            error_z * self.KP_PITCH, -self.uav.p.max_pitch_rate_dps, self.uav.p.max_pitch_rate_dps
        )
        leveling_pitch = -self.uav.s.pitch * 0.5
        self.uav.cmd_pitch_rate = target_pitch_rate + leveling_pitch

        target_yaw_deg = wrap_deg(math.degrees(math.atan2(-dy, dx)))
        # Blend toward outbound leg only when heading to the actual waypoint (not helper points).
        if (not self.rejoin_points) and self.entry_heading_deg is not None:
            blend_t = clamp(dist_2d / ENTRY_BLEND_DIST_M, 0.0, 1.0)
            delta = _angle_diff_deg(target_yaw_deg, self.entry_heading_deg)
            target_yaw_deg = wrap_deg(self.entry_heading_deg + delta * blend_t)
        if self.heading_override_deg is not None:
            target_yaw_deg = self.heading_override_deg
        error_yaw_deg = _angle_diff_deg(target_yaw_deg, self.uav.s.yaw)
        self.uav.cmd_yaw_rate = clamp(
            error_yaw_deg * self.KP_YAW, -self.uav.p.max_yaw_rate_dps, self.uav.p.max_yaw_rate_dps
        )

        target_roll_deg = clamp(self.uav.s.r * 1.5, -30.0, 30.0)
        error_roll_deg = target_roll_deg - self.uav.s.roll
        self.uav.cmd_roll_rate = clamp(
            error_roll_deg * self.KP_ROLL, -self.uav.p.max_roll_rate_dps, self.uav.p.max_roll_rate_dps
        )

        error_speed = self.target_speed - self.uav.s.u
        self.uav.cmd_throttle = clamp(error_speed * self.KP_THROTTLE, -1.0, 1.0)

    def _update_gimbal(self, dt: float):
        if self.current_start is None or self.current_end is None or self.scan_duration <= 0.0 or self.mission_done:
            return
        self.scan_timer += dt
        t = clamp(self.scan_timer / self.scan_duration, 0.0, 1.0)
        self.current_gimbal_target = self.current_start * (1.0 - t) + self.current_end * t
        if t >= 1.0:
            self.scan_duration = 0.0

    def update(self, dt: float):
        """Run one control tick; returns the current gimbal target (np.ndarray or None)."""
        self._update_movement(dt)
        self._update_gimbal(dt)
        return self.current_gimbal_target


def build_standoff_controller(uav, dem, target_speed: float = TARGET_SPEED) -> StandoffController | None:
    packages = _build_packages()
    if not packages:
        return None
    return StandoffController(uav=uav, dem=dem, packages=packages, target_speed=target_speed)
