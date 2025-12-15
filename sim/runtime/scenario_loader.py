from __future__ import annotations

import json
import math
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

from sim.entities.entities import MovingTarget
from sim.entities.threat import AirDefenseThreat
from sim.runtime.constants import TARGET_ROAM_RADIUS_M, TARGET_SPEED_RANGE, THREAT_RADAR, THREAT_WEAPON
from sim.runtime.autopilot import Mission, Waypoint
from sim.world.dem import DEM


@dataclass
class ScenarioData:
    uav_positions: List[Tuple[float, float, float]]
    lah_positions: List[Tuple[float, float, float]]
    targets: List[MovingTarget]


def _latest_json(path: Path) -> Path | None:
    if not path.exists():
        return None
    files = sorted([p for p in path.glob("*.json") if p.is_file()], key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0] if files else None


def _lonlat_to_env(dem: DEM, lon: float, lat: float, alt: float | None) -> Tuple[float, float, float]:
    x, y = dem.lonlat_to_env(lon, lat)
    ground_z = dem.get_height(x, y)
    z = alt if alt is not None else ground_z
    return x, y, z


def _load_uav_positions(mref_path: Path, dem: DEM) -> List[Tuple[float, float, float]]:
    data = json.loads(mref_path.read_text(encoding="utf-8"))
    takeovers = data.get("takeOverInfoList") or []
    positions: dict[int, Tuple[float, float, float]] = {}
    for entry in takeovers:
        try:
            aid = int(entry.get("aircraftID"))
            coord = entry.get("coordinate") or {}
            lat = float(coord.get("latitude"))
            lon = float(coord.get("longitude"))
            alt = coord.get("altitude")
        except Exception:
            continue
        if aid not in (4, 5, 6):
            continue
        pos = _lonlat_to_env(dem, lon, lat, alt)
        positions[aid] = pos
    return [positions.get(aid) for aid in (4, 5, 6) if positions.get(aid)]


def _build_lah_positions(uav_positions: Iterable[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
    lahs: List[Tuple[float, float, float]] = []
    for pos in uav_positions:
        x, y, z = pos
        lahs.append((x, y - 300.0, z))
        if len(lahs) >= 3:
            break
    return lahs


def _load_targets(target_path: Path, dem: DEM) -> List[MovingTarget]:
    data = json.loads(target_path.read_text(encoding="utf-8"))
    tlist = data.get("targetList") or []
    targets: List[MovingTarget] = []
    wxmin, wxmax, wymin, wymax = dem.get_world_env_bounds()
    world_half = max(abs(wxmin), abs(wxmax), abs(wymin), abs(wymax))
    # Keep IDs unique by offsetting by 6 (avoid aircraftID overlap).
    for entry in tlist:
        loc = entry.get("location") or {}
        try:
            lat = float(loc.get("latitude"))
            lon = float(loc.get("longitude"))
            alt = loc.get("altitude") or 0.0
        except Exception:
            continue
        x, y, _ = _lonlat_to_env(dem, lon, lat, alt)
        tgt = MovingTarget(
            x=x,
            y=y,
            vmin=TARGET_SPEED_RANGE[0],
            vmax=TARGET_SPEED_RANGE[1],
            roam_center=(x, y),
            roam_radius=TARGET_ROAM_RADIUS_M,
            threat=AirDefenseThreat(radar=THREAT_RADAR, weapon=THREAT_WEAPON),
            world_half=world_half,
        )
        tgt.id = int(entry.get("targetID", len(targets) + 1)) + 6
        targets.append(tgt)
    return targets


def load_scenario(db_root: Path, dem: DEM) -> ScenarioData | None:
    """
    Load a scenario from the database folder.
    - UAV takeoff positions from MissionReferenceInfo (aircraftID 4/5/6).
    - LAH positions 300m south of each UAV.
    - Targets from TargetInfo (IDs offset by +6).
    Returns None if required files are missing.
    """
    db_root = db_root.expanduser().resolve()
    mref_dir = db_root / "MissionReferenceInfo"
    tgt_dir = db_root / "TargetInfo"

    mref = _latest_json(mref_dir)
    tgt = _latest_json(tgt_dir)
    if mref is None or tgt is None:
        return None

    uav_positions = _load_uav_positions(mref, dem)
    lah_positions = _build_lah_positions(uav_positions)
    targets = _load_targets(tgt, dem)
    return ScenarioData(uav_positions=uav_positions, lah_positions=lah_positions, targets=targets)


def apply_scenario_to_state(state, scenario: ScenarioData, dem: DEM) -> None:
    """Mutate SimulationState with loaded scenario (positions + targets)."""
    if not scenario:
        return

    # Place LAHs (indexes 0-2)
    for idx, pos in enumerate(scenario.lah_positions):
        if idx >= len(state.uavs):
            break
        with state.uav_locks[idx]:
            u = state.uavs[idx]
            u.s.x, u.s.y, u.s.z = pos
            state.initial_spawn_points[idx] = pos

    # Place UAVs (aircraftID 4/5/6 -> indexes 3/4/5)
    for offset, pos in enumerate(scenario.uav_positions):
        idx = 3 + offset
        if idx >= len(state.uavs):
            break
        with state.uav_locks[idx]:
            u = state.uavs[idx]
            u.s.x, u.s.y, u.s.z = pos
            state.initial_spawn_points[idx] = pos

    # Targets
    if scenario.targets:
        state.targets = scenario.targets

    # Clamp any spawn inside DEM bounds
    xmin, xmax, ymin, ymax = dem.get_world_env_bounds()
    for idx, pos in enumerate(state.initial_spawn_points):
        if idx >= len(state.uavs):
            break
        x, y, z = pos
        x = max(xmin, min(xmax, x))
        y = max(ymin, min(ymax, y))
        z = max(dem.get_height(x, y), z)
        state.initial_spawn_points[idx] = (x, y, z)
        with state.uav_locks[idx]:
            state.uavs[idx].s.x, state.uavs[idx].s.y, state.uavs[idx].s.z = state.initial_spawn_points[idx]


def peek_reference_coords(db_root: Path) -> list[tuple[float, float, float]]:
    """Return lon/lat/alt tuples from the latest MissionReferenceInfo file (aircraft 4/5/6)."""
    db_root = db_root.expanduser().resolve()
    mref = _latest_json(db_root / "MissionReferenceInfo")
    if mref is None:
        return []
    data = json.loads(mref.read_text(encoding="utf-8"))
    coords = []
    for entry in data.get("takeOverInfoList") or []:
        try:
            aid = int(entry.get("aircraftID"))
            if aid not in (4, 5, 6):
                continue
            c = entry.get("coordinate") or {}
            lat = float(c.get("latitude"))
            lon = float(c.get("longitude"))
            alt = float(c.get("altitude", 0.0))
            coords.append((lon, lat, alt))
        except Exception:
            continue
    return coords


def pick_dem_for_coords(coords: Sequence[tuple[float, float, float]], map_dir: Path) -> Path | None:
    """
    Choose a DEM tile from map_dir that covers the first provided lon/lat.
    Tiles are expected to be named like n37_e127_*.tif (1° grid).
    """
    if not coords:
        return None
    tiles = {}
    for tif in sorted(map_dir.glob("*.tif")):
        m = re.search(r"([ns])(\d+)_([ew])(\d+)", tif.stem.lower())
        if not m:
            continue
        lat = int(m.group(2))
        if m.group(1) == "s":
            lat = -lat
        lon = int(m.group(4))
        if m.group(3) == "w":
            lon = -lon
        tiles[(lat, lon)] = tif
    for lon, lat, _ in coords:
        lat_deg = math.floor(lat)
        lon_deg = math.floor(lon)
        candidate = tiles.get((lat_deg, lon_deg))
        if candidate:
            return candidate
    return None


def _collect_flightpath_files(fp_dir: Path, aircraft_id: int) -> list[tuple[int | None, float, Path]]:
    """
    Collect all flightpath files for an aircraft, sorted by pathID (asc) then mtime.
    Returns list of (pathID, mtime, path).
    """
    files: list[tuple[int | None, float, Path]] = []
    for p in fp_dir.glob("*.json"):
        if not p.is_file():
            continue
        try:
            data = json.loads(p.read_text(encoding="utf-8"))
        except Exception:
            continue
        if int(data.get("aircraftID", -1)) != aircraft_id:
            continue
        pid = data.get("pathID")
        pid_int = int(pid) if isinstance(pid, (int, float, str)) and str(pid).isdigit() else None
        files.append((pid_int, p.stat().st_mtime, p))
    files.sort(key=lambda t: (t[0] if t[0] is not None else 0, t[1]))
    return files


def _waypoints_from_flightpath(path: Path, dem: DEM, aircraft_id: int | None = None) -> list[Waypoint]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return []
    # LAH 파일은 lahWaypointList를 쓸 수 있고, UAV는 waypointList를 사용.
    if aircraft_id is None:
        try:
            aircraft_id = int(data.get("aircraftID"))
        except Exception:
            aircraft_id = None
    waypoints_json = []
    if aircraft_id is not None and aircraft_id <= 3:
        waypoints_json = data.get("lahWaypointList") or data.get("waypointList") or []
    else:
        waypoints_json = data.get("waypointList") or data.get("lahWaypointList") or []
    waypoints: List[Waypoint] = []
    for idx, wp in enumerate(waypoints_json):
        coord = wp.get("coordinate") or {}
        try:
            lat = float(coord.get("latitude"))
            lon = float(coord.get("longitude"))
        except Exception:
            continue
        alt = coord.get("altitude")
        x, y = dem.lonlat_to_env(lon, lat)
        z = float(alt) if alt is not None else dem.get_height(x, y)
        line_search = None
        search_speed = None
        fp = wp.get("filmingProperty") or {}
        ls = fp.get("lineSearch") or {}
        coords = ls.get("coordinateList") or []
        if coords and isinstance(coords, list):
            line_search = []
            for c in coords:
                try:
                    lat_ls = float(c.get("latitude"))
                    lon_ls = float(c.get("longitude"))
                    alt_ls = c.get("altitude")
                except Exception:
                    continue
                x_ls, y_ls = dem.lonlat_to_env(lon_ls, lat_ls)
                z_ls = float(alt_ls) if alt_ls is not None else dem.get_height(x_ls, y_ls)
                line_search.append((x_ls, y_ls, z_ls))
        if ls.get("searchSpeed") is not None:
            try:
                search_speed = float(ls.get("searchSpeed"))
            except Exception:
                search_speed = None
        wp_speed = None
        if "speed" in wp:
            try:
                wp_speed = float(wp.get("speed"))
            except Exception:
                wp_speed = None
        waypoints.append(
            Waypoint(
                x=x,
                y=y,
                z=z,
                name=f"wp{idx + 1}",
                line_search=line_search,
                search_speed=search_speed,
                speed=wp_speed,
            )
        )
    return waypoints


def load_flightpaths(db_root: Path, dem: DEM, aircraft_ids: Sequence[int] = (1, 2, 3, 4, 5, 6)) -> dict[int, Mission]:
    """Load all flightpaths per aircraft ID, concatenating waypoints in pathID order."""
    fp_dir = db_root.expanduser().resolve() / "FlightPath"
    missions: dict[int, Mission] = {}
    for aid in aircraft_ids:
        files = _collect_flightpath_files(fp_dir, aid)
        if not files:
            print(f"[flightpath] aircraftID {aid}: no file found in {fp_dir}")
            continue
        all_wps: list[Waypoint] = []
        summary = []
        for pid, mtime, path in files:
            wps = _waypoints_from_flightpath(path, dem, aircraft_id=aid)
            all_wps.extend(wps)
            summary.append((pid, path.name, len(wps)))
        if not all_wps:
            print(f"[flightpath] aircraftID {aid}: found {len(files)} files but no waypoints parsed.")
            continue
        missions[aid] = Mission(waypoints=all_wps, current_idx=0, active=True, completed=False)
        detail = ", ".join([f"{pid or '?'}:{name}({cnt})" for pid, name, cnt in summary[:5]])
        print(f"[flightpath] aircraftID {aid}: loaded {len(all_wps)} wps from {len(files)} files -> {detail}")
    return missions
