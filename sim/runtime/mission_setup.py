from __future__ import annotations

import json
import random
from typing import TYPE_CHECKING

from sim.config import MAP_DIR, RENDER_RADIUS_M

if TYPE_CHECKING:
    from sim.runtime.app import SimulationApp


def log_dem_info(dem) -> None:
    """DEM 정보 출력."""
    print(f"[DEM] tiles dir: {MAP_DIR}")
    print(
        f"[DEM] shape: {dem.elevation.shape} lon/lat bounds: ({dem.xmin:.5f},{dem.ymin:.5f})-({dem.xmax:.5f},{dem.ymax:.5f})"
    )
    print(f"[DEM] elevation min/max: {dem.min_elev:.1f}/{dem.max_elev:.1f}")


def apply_mission_reference_spawns(app: "SimulationApp") -> None:
    """미션 레퍼런스 파일을 읽어 항공기 초기 위치를 배치."""
    if not app.mission_reference_path:
        return
    if not app.mission_reference_path.exists():
        print(f"[mission-ref] file not found: {app.mission_reference_path}")
        return
    try:
        data = json.loads(app.mission_reference_path.read_text(encoding="utf-8"))
    except Exception as e:
        print(f"[mission-ref] failed to load {app.mission_reference_path}: {e}")
        return
    take_over_map = {item.get("aircraftID"): item.get("coordinate") for item in data.get("takeOverInfoList", [])}
    rtb_list = data.get("rtbCoordinateList", [])

    def _env_from_coord(coord):
        lon = coord.get("longitude")
        lat = coord.get("latitude")
        alt = coord.get("altitude", 0.0)
        x, y = app.dem.lonlat_to_env(lon, lat)
        # Ensure tile switches if needed (propagate if out of bounds)
        app.dem.ensure_tile_for_env(x, y)
        ground = app.dem.get_height(x, y)
        return x, y, alt, ground

    # First pass: compute env coords for each craft; find anchor from first non-LAH craft.
    env_coords = []
    anchor_spawn = None
    for idx, uav_type in enumerate(app.state.uav_types):
        aircraft_id = idx + 1
        coord = take_over_map.get(aircraft_id)
        if coord is None and rtb_list:
            coord = rtb_list[min(idx, len(rtb_list) - 1)]
        if coord is None:
            env_coords.append(app.state.initial_spawn_points[idx])
            continue
        x, y, alt, ground = _env_from_coord(coord)
        env_coords.append((x, y, alt, ground))
        if anchor_spawn is None and uav_type != "LAH":
            anchor_spawn = (x, y, ground)

    # Fallback anchor: use first craft (even LAH) if no non-LAH found.
    if anchor_spawn is None and env_coords:
        ex, ey, ealt, eg = env_coords[0]
        anchor_spawn = (ex, ey, eg)

    new_spawns = []
    for idx, uav in enumerate(app.state.uavs):
        uav_type = app.state.uav_types[idx]
        if idx >= len(env_coords):
            new_spawns.append(app.state.initial_spawn_points[idx])
            continue
        x, y, alt, ground = env_coords[idx]
        if uav_type == "LAH" and anchor_spawn is not None:
            ax, ay, ag = anchor_spawn
            x = ax + random.uniform(-200.0, 200.0)
            y = ay + random.uniform(-200.0, 200.0)
            ground = app.dem.get_height(x, y)
            z = ground + 100.0
        else:
            z = alt if alt > 0 else ground + 50.0
        uav.s.x, uav.s.y, uav.s.z = x, y, z
        new_spawns.append((x, y, z))

    # Update stored spawn points so reset() keeps them.
    app.state.initial_spawn_points = new_spawns


def relocate_targets_near_spawn(app: "SimulationApp") -> None:
    """초기 스폰 근처로 타깃을 옮겨 DEM/LOS 부하 감소."""
    if not app.state.targets or not app.state.initial_spawn_points:
        return
    cx, cy, cz = app.state.initial_spawn_points[0]
    for t in app.state.targets:
        ox = random.uniform(-200.0, 200.0)
        oy = random.uniform(-200.0, 200.0)
        tx = cx + ox
        ty = cy + oy
        tz = app.dem.get_height(tx, ty) + 50.0
        t.x = tx
        t.y = ty
        t.z = tz
        if hasattr(t, "roam_center"):
            t.roam_center = (tx, ty)


def ensure_spawn_above_dem(app: "SimulationApp") -> None:
    """미션 파일이 없을 때 지면 위로 스폰 위치 보정."""
    if not app.state or not app.state.initial_spawn_points:
        return
    new_spawns = []
    for idx, uav in enumerate(app.state.uavs):
        x, y, z = app.state.initial_spawn_points[idx]
        ground = app.dem.get_height(x, y)
        # LAH higher buffer, UAV lower buffer
        if app.state.uav_types[idx] == "LAH":
            z = max(z, ground + 120.0)
        else:
            z = max(z, ground + 80.0)
        uav.s.x, uav.s.y, uav.s.z = x, y, z
        new_spawns.append((x, y, z))
    app.state.initial_spawn_points = new_spawns


def load_flight_paths(app: "SimulationApp") -> None:
    """로그에서 비행 경로 로드 후 초기 상태 보정."""
    if not app.flight_path_root.exists():
        print(f"[flight-path] directory not found: {app.flight_path_root}")
        return
    paths_per_aircraft: list[list[dict]] = [[] for _ in app.state.uavs]

    def craft_idx_from_path_id(pid: int) -> int | None:
        prefix = int(str(pid)[0])
        mapping = {1: 0, 2: 1, 3: 2, 4: 3, 5: 4, 6: 5}
        return mapping.get(prefix)

    counts = [0 for _ in app.state.uavs]
    for fp in sorted(app.flight_path_root.glob("*.json")):
        try:
            data = json.loads(fp.read_text(encoding="utf-8"))
        except Exception as e:
            print(f"[flight-path] failed to load {fp}: {e}")
            continue
        pid = data.get("pathID")
        if pid is None:
            continue
        idx = craft_idx_from_path_id(int(pid))
        if idx is None or idx >= len(paths_per_aircraft):
            continue
        # Allow both waypointList (UAV) and lahWaypointList (LAH)
        wps = data.get("waypointList", []) or data.get("lahWaypointList", [])
        for wp in wps:
            coord = wp.get("coordinate") or {}
            lon = coord.get("longitude")
            lat = coord.get("latitude")
            alt = coord.get("altitude", 0.0)
            speed = wp.get("speed")
            hover = (wp.get("hovering") or {}).get("time")
            if lon is None or lat is None:
                continue
            try:
                x, y = app.dem.lonlat_to_env(lon, lat)
                z = app.dem.get_height(x, y) if alt == 0 else alt
            except Exception as e:
                print(f"[flight-path] coord out of DEM bounds for {fp}: {e}")
                continue
            paths_per_aircraft[idx].append(
                {
                    "pos": (x, y, z),
                    "wp_id": wp.get("waypointID"),
                    "path_id": pid,
                    "speed": speed,
                    "filming": wp.get("filmingProperty"),
                    "hover_time": hover,
                    "loiter": wp.get("loiterProperty"),
                    "hover_prop": wp.get("hover_prop") or wp.get("hovering"),
                }
            )
            counts[idx] += 1
    app.state.flight_paths = paths_per_aircraft
    # Lift initial spawn to first waypoint to avoid ground collision (especially for LAH with z=100 default).
    for idx, plist in enumerate(paths_per_aircraft):
        if not plist:
            continue
        first = plist[0]
        pos = first.get("pos")
        if not pos or len(pos) != 3:
            continue
        with app.state.uav_locks[idx]:
            try:
                fx, fy, fz = map(float, pos)
                app.state.uavs[idx].s.x = fx
                app.state.uavs[idx].s.y = fy
                app.state.uavs[idx].s.z = max(fz, app.dem.get_height(fx, fy) + 5.0)
                # Give rotorcraft a small forward speed to avoid stall/ground clamp on start.
                if app.state.uav_types[idx] == "LAH":
                    try:
                        first_speed = float(first.get("speed") or 0.0)
                    except Exception:
                        first_speed = 0.0
                    if first_speed > 0.0:
                        app.state.uavs[idx].s.u = max(5.0, first_speed * 0.25)
                    else:
                        app.state.uavs[idx].s.u = 0.0
            except Exception:
                pass
        # Keep reset position in sync.
        if idx < len(app.state.initial_spawn_points):
            app.state.initial_spawn_points[idx] = (
                app.state.uavs[idx].s.x,
                app.state.uavs[idx].s.y,
                app.state.uavs[idx].s.z,
            )
    for idx, cnt in enumerate(counts):
        if cnt > 0:
            name = "LAH" if idx < 3 else "UAV"
            print(f"[flight-path] loaded {cnt} waypoints for {name}{idx+1}")
