import math
import random
from datetime import datetime, timezone
from typing import Optional, Tuple, List

import numpy as np

from sim.world.dem import check_los, ray_intersect_dem

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)

SOURCE_NAME = "test"
MAX_FUEL_L = 1000.0  # spec max
# 2-hour endurance target -> burn the full tank over ~7200s
LAH_FUEL_BURN_LPS = MAX_FUEL_L / (2 * 3600.0)
DEFAULT_FOV_ASPECT = 16 / 9


def now_ms_2000() -> int:
    return int((datetime.now(timezone.utc) - _EPOCH_2000).total_seconds() * 1000)


def _datalink_status_for_lah(lah_pos, snapshots, dem):
    """Return LOS flags to UAV1/2/3 (indexes 3,4,5)."""

    def _safe_snap(idx):
        return snapshots[idx] if 0 <= idx < len(snapshots) else None

    los_flags = []
    for tgt_idx in (3, 4, 5):
        tgt = _safe_snap(tgt_idx)
        if tgt is None:
            los_flags.append(False)
            continue
        los = check_los(np.array(lah_pos, dtype=float), np.array(tgt[:3], dtype=float), dem)
        los_flags.append(bool(los))
    return {
        "isConnectedToUAV1": los_flags[0],
        "isConnectedToUAV2": los_flags[1],
        "isConnectedToUAV3": los_flags[2],
    }


def _leader_aircraft_id(crashed: list[bool]) -> int:
    if len(crashed) >= 6:
        if not crashed[3]:
            return 4
        if not crashed[4]:
            return 5
        if not crashed[5]:
            return 6
    return 4 if len(crashed) >= 4 else 1


def _fuel_warning(fuel_l: float) -> int:
    pct = fuel_l / MAX_FUEL_L if MAX_FUEL_L > 0 else 0.0
    if pct <= 0.01:
        return 3
    if pct <= 0.2:
        return 2
    return 1


def _compute_footprint(uav_pos, tgt_pos, fov_diag_deg, dem, aspect_ratio=DEFAULT_FOV_ASPECT):
    if tgt_pos is None:
        return [], None
    uav_pos = np.array(uav_pos, dtype=float)
    forward = np.array(tgt_pos, dtype=float) - uav_pos
    if np.linalg.norm(forward) < 1e-6:
        return [], None
    forward /= np.linalg.norm(forward)

    fov_diag = math.radians(fov_diag_deg)
    ar = aspect_ratio
    fov_h = 2 * math.atan(math.tan(fov_diag / 2) * ar / math.sqrt(1 + ar**2))
    fov_v = 2 * math.atan(math.tan(fov_diag / 2) * 1 / math.sqrt(1 + ar**2))

    world_up = np.array([0, 0, 1], dtype=float)
    right = np.cross(forward, world_up)
    if np.linalg.norm(right) < 1e-6:
        right = np.array([1, 0, 0], dtype=float)
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    up /= np.linalg.norm(up)

    tan_h, tan_v = math.tan(fov_h / 2), math.tan(fov_v / 2)
    combos = [(-1, 1), (1, 1), (1, -1), (-1, -1)]  # TL, TR, BR, BL
    corners_env = []
    for sx, sy in combos:
        d = forward + sx * tan_h * right + sy * tan_v * up
        d = d / np.linalg.norm(d)
        hit = ray_intersect_dem(uav_pos, d, dem)
        if hit is None:
            return [], None
        corners_env.append(hit)

    footprint_lonlat = []
    for c in corners_env:
        lon, lat = dem.env_to_lonlat(c[0], c[1])
        footprint_lonlat.append({"latitude": lat, "longitude": lon, "altitude": float(c[2])})

    tgt_lon, tgt_lat = dem.env_to_lonlat(tgt_pos[0], tgt_pos[1])
    center_coord = {"latitude": tgt_lat, "longitude": tgt_lon, "altitude": float(tgt_pos[2])}
    return footprint_lonlat, center_coord


def _make_agent_state(idx, snap, uav_type, fuel_l, crashed, crippled, snapshots, dem, targets, fov_diag):
    lon, lat = dem.env_to_lonlat(snap[0], snap[1])
    health = 2 if crashed[idx] or crippled[idx] else 1
    is_unmanned = uav_type != "LAH"

    target_pos: Optional[Tuple[float, float, float]] = None
    target_id: Optional[int] = None
    if targets:
        target = min(targets, key=lambda T: (T.x - snap[0]) ** 2 + (T.y - snap[1]) ** 2 + (T.z - snap[2]) ** 2)
        target_pos = (target.x, target.y, target.z)
        target_id = getattr(target, "id", None) or targets.index(target) + 1

    footprint_list, center_coord = _compute_footprint(snap[:3], target_pos, fov_diag, dem)
    sensor_info = {
        "operationalMode": 1,
        "sensorType": 2,
        "fov": float(fov_diag),
        "centerCoordinate": center_coord or {"latitude": lat, "longitude": lon, "altitude": int(snap[2])},
    }
    if footprint_list:
        sensor_info["footprintCornerList"] = footprint_list

    leader_id = _leader_aircraft_id(crashed)
    manned_info = None
    if not is_unmanned:
        manned_info = {
            "weapons": {"type1": 10, "type2": 10, "type3": 10},
            "datalinkStatus": _datalink_status_for_lah(snap[:3], snapshots, dem),
        }
    else:
        fuel_warn = _fuel_warning(fuel_l)
        unmanned_info = {
            "currentWaypointID": {"waypointID": 0},
            "flightMode": 7,
            "leaderAircraftID": {"aircraftID": leader_id},
            "sensorInfo": sensor_info,
            "payloadHealth": 2,
            "fuelWarning": fuel_warn,
        }
        if target_id is not None:
            unmanned_info["targetFollowing"] = {"targetID": target_id}
    state = {
        "aircraftID": idx + 1,
        "isUnmanned": is_unmanned,
        "coordinate": {"latitude": lat, "longitude": lon, "altitude": int(snap[2])},
        "velocity": {"speed": float(snap[6]), "heading": float(snap[5])},
        "fuel": max(0.0, min(fuel_l, MAX_FUEL_L)),
        "health": health,
        "mannedInfo": manned_info
        or {
            "weapons": {"type1": 0, "type2": 0, "type3": 0},
            "datalinkStatus": {
                "isConnectedToUAV1": False,
                "isConnectedToUAV2": False,
                "isConnectedToUAV3": False,
            },
        },
        "unmannedInfo": unmanned_info if is_unmanned else {},
    }
    return state


def build_agent_status_snapshot(uav_types, snapshots, fuel_levels, crashed, crippled, dem, targets, fov_diag):
    agent_states = []
    for i, snap in enumerate(snapshots):
        fuel = fuel_levels[i] if i < len(fuel_levels) else 0.0
        agent_states.append(
            _make_agent_state(i, snap, uav_types[i], fuel, crashed, crippled, snapshots, dem, targets, fov_diag)
        )
    return {"timestamp": now_ms_2000(), "source": SOURCE_NAME, "agentStateList": agent_states}
