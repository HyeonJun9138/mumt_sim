import math
import random
from dataclasses import dataclass
from typing import List, Sequence

from sim.config import clamp, wrap_deg
from sim.world.dem import DEM


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    name: str
    line_search: list[tuple[float, float, float]] | None = None
    search_speed: float | None = None
    speed: float | None = None


@dataclass
class Mission:
    waypoints: List[Waypoint]
    current_idx: int = 0
    active: bool = False
    completed: bool = False


def get_dta(V: float, max_roll: float, leg_deg: float, dlim: float = 800.0) -> float:
    """
    Distance-to-alternate: distance before a waypoint to begin turning to the next leg.
    Mirrors the reference logic provided (units: meters, degrees, m/s).
    """
    V = max(V, 1e-3)
    max_roll = max(1e-3, max_roll)

    # Cap bank based on speed and turn angle
    max_roll = min(max_roll, V * 0.9719)
    max_roll = min(max_roll, abs(leg_deg) * 0.5 if leg_deg else max_roll)

    # Turn radius
    Rv = V**2 / (9.8 * max(1e-3, math.tan(math.radians(max_roll))))

    dta = Rv * min(math.tan(math.radians(abs(leg_deg) * 0.5)), 8.0) + 3.0 * V
    return min(dta, dlim)


def get_radius(V: float, max_roll: float) -> float:
    return V**2 / (9.8 * max(1e-3, math.tan(math.radians(max_roll))))


def _make_curve_waypoints(origin, dem: DEM, count: int = 5, base_radius: float = 1200.0) -> List[Waypoint]:
    """
    Build a mostly-straight, long 5-WP path around the origin, each 300m AGL.
    """
    ox, oy, _ = origin
    angle = random.uniform(-math.pi, math.pi)
    wps: List[Waypoint] = []
    for i in range(count):
        # Larger spacing with gentle heading change to avoid zig-zagging
        radius = base_radius + 400.0 * i
        angle += math.radians(random.uniform(5.0, 25.0))
        x = ox + math.cos(angle) * radius
        y = oy + math.sin(angle) * radius
        ground = dem.get_height(x, y)
        z = ground + 300.0
        wps.append(Waypoint(x=x, y=y, z=z, name=f"wp{i + 1}"))
    return wps


def build_missions(origins: Sequence[tuple[float, float, float]], dem: DEM) -> List[Mission]:
    missions: List[Mission] = []
    for origin in origins:
        wps = _make_curve_waypoints(origin, dem)
        missions.append(Mission(waypoints=wps, current_idx=0, active=True, completed=False))
    return missions


def _advance_if_reached(mission: Mission, uav_pos, dta: float, xy_tol: float = 25.0, z_tol: float = 15.0):
    wp = mission.waypoints[mission.current_idx]
    dx = wp.x - uav_pos[0]
    dy = wp.y - uav_pos[1]
    dz = wp.z - uav_pos[2]
    horiz = math.hypot(dx, dy)
    # Allow larger pass-by window based on DTA (start next leg early)
    passby = max(xy_tol, min(150.0, dta * 0.6))
    if horiz <= passby and abs(dz) <= z_tol:
        mission.current_idx += 1
        if mission.current_idx >= len(mission.waypoints):
            mission.completed = True
            mission.active = False
            mission.current_idx = len(mission.waypoints) - 1


def _command_autopilot(uav, target_wp: Waypoint, is_lah: bool):
    yaw_to_wp = math.degrees(math.atan2(-(target_wp.y - uav.s.y), target_wp.x - uav.s.x))
    yaw_err = wrap_deg(yaw_to_wp - uav.s.yaw)
    yaw_k = 0.9
    uav.cmd_yaw_rate = clamp(yaw_err * yaw_k, -uav.p.max_yaw_rate_dps, uav.p.max_yaw_rate_dps)

    # Level out while turning
    uav.cmd_roll_rate = -uav.s.roll * 1.5

    # Altitude control via pitch rate
    alt_err = target_wp.z - uav.s.z
    pitch_k = 0.12 if is_lah else 0.08
    uav.cmd_pitch_rate = clamp(
        alt_err * pitch_k, -uav.p.max_pitch_rate_dps * 0.6, uav.p.max_pitch_rate_dps * 0.6
    )

    # Simple speed hold
    wp_speed = getattr(target_wp, "speed", None)
    target_speed = float(wp_speed) if wp_speed is not None else (40.0 if is_lah else 90.0)
    speed_err = target_speed - uav.s.u
    throttle_k = 1.0 / max(uav.p.accel, 1e-3)
    uav.cmd_throttle = clamp(speed_err * throttle_k, -1.0, 1.0)


def update_autopilot(state, dem: DEM, dt: float):
    if not getattr(state, "autopilot_enabled", False):
        return
    for idx, uav in enumerate(state.uavs):
        if state.crashed[idx] or state.crippled[idx]:
            continue
        mission = state.missions[idx] if idx < len(state.missions) else None
        if mission is None or mission.completed or not mission.active:
            continue
        wp = mission.waypoints[mission.current_idx]
        next_wp = mission.waypoints[mission.current_idx + 1] if mission.current_idx + 1 < len(mission.waypoints) else None

        # Heading change between inbound leg and outbound leg
        if next_wp:
            inbound_deg = math.degrees(math.atan2(-(wp.y - uav.s.y), wp.x - uav.s.x))
            outbound_deg = math.degrees(math.atan2(-(next_wp.y - wp.y), next_wp.x - wp.x))
            leg_deg = wrap_deg(outbound_deg - inbound_deg)
            V = max(uav.s.u, 1e-3)
            max_roll = getattr(uav.p, "roll_limit_deg", 30.0)
            dta = get_dta(V, max_roll, leg_deg)
        else:
            leg_deg = 0.0
            dta = 0.0

        # If within DTA and there is a next leg, start aiming at the next waypoint early.
        active_target = next_wp if (next_wp and math.hypot(wp.x - uav.s.x, wp.y - uav.s.y) <= dta) else wp

        with state.uav_locks[idx]:
            _command_autopilot(uav, active_target, is_lah=state.uav_types[idx] == "LAH")
            scan_active = False
            if hasattr(state, "line_scan_states") and state.line_scan_states:
                s = state.line_scan_states[idx] if idx < len(state.line_scan_states) else None
                scan_active = s is not None and not getattr(s, "finished", False)
            if not scan_active:
                _advance_if_reached(mission, (uav.s.x, uav.s.y, uav.s.z), dta=dta)
