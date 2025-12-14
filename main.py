'''

 --- 조작 키 안내 ---
# W/S: 스로틀 업/다운
# 방향키 좌/우: 요(회전) + 롤 입력, 방향키 상/하: 피치 업/다운
# SPACE: 기체 자세 수평으로 복귀
# R: UAV 상태 리셋
# E/D: 카메라 대각 FOV 확대/축소
# M: 가장 가까운 타겟으로 미사일 발사
# N: 타겟 이동 on/off 토글
# G: 카메라 기준 주변에 타겟 하나 추가 생성
# 마우스 휠: 줌(카메라 거리 조정)
# 마우스 오른쪽 드래그: 궤도(orbit) 회전, 중간 버튼 드래그: 패닝
# ESC/Q: 종료

'''


import math
import os
import json
import random
import threading
import time
import numpy as np
import pygame
from pygame.locals import DOUBLEBUF, OPENGL
from OpenGL.GL import (
    glBegin,
    glBlendFunc,
    glClear,
    glClearColor,
    glColor3f,
    glColor4f,
    glDisable,
    glEnable,
    glEnd,
    glFogf,
    glFogfv,
    glFogi,
    glHint,
    glLineWidth,
    glLineStipple,
    glLoadIdentity,
    glMatrixMode,
    GL_MODELVIEW,
    GL_PROJECTION,
    glVertex3f,
    glViewport,
    GL_BLEND,
    GL_COLOR_BUFFER_BIT,
    GL_DEPTH_BUFFER_BIT,
    GL_DEPTH_TEST,
    GL_EXP2,
    GL_FOG,
    GL_FOG_COLOR,
    GL_FOG_DENSITY,
    GL_FOG_HINT,
    GL_FOG_MODE,
    GL_LINE_LOOP,
    GL_LINES,
    GL_LINE_STRIP,
    GL_LINE_STIPPLE,
    GL_NICEST,
    GL_SRC_ALPHA,
    GL_ONE_MINUS_SRC_ALPHA,
    GLfloat,
    glGetString,
    GL_VENDOR,
    GL_RENDERER,
    GL_VERSION,
    GL_SHADING_LANGUAGE_VERSION,
    glGetError,
    glGetDoublev,
    glGetIntegerv,
    GL_MODELVIEW_MATRIX,
    GL_PROJECTION_MATRIX,
    GL_VIEWPORT,
)
from OpenGL.GLU import gluPerspective, gluProject

from sim.core.camera import OrbitCamera
from sim.config import (
    CLEAR_COLOR,
    DEFAULT_FOV_DIAG,
    DEM_FILE,
    REPO_ROOT,
    FPS,
    FOG_COLOR,
    FOG_DENSITY,
    WIN_W,
    WIN_H,
    WORLD_HALF,
    RENDER_RADIUS_M,
    DEM_PREFETCH_RADIUS_M,
    DEM_CACHE_MOVE_THRESHOLD_M,
    clamp,
)
from sim.world.dem import DEM, check_los, ray_intersect_dem
from sim.render.draw import (
    draw_axes,
    draw_camera_footprint,
    draw_dem,
    draw_grid,
    draw_hud,
    draw_uav,
    draw_labels,
    draw_lah,
)
from sim.entities.entities import MovingTarget, Missile
from sim.entities.threat import AirDefenseThreat, RadarParams, WeaponParams, WeaponType
from sim.core.uav import UAV, UAVParams
from sim.core.lah import LAH, LAHParams

TARGET_SPAWNS = [
    # (x, y) in meters; edit this list to pre-place multiple targets
    (-200.0, -150.0),
    (250.0, 180.0),
]
TARGET_ROAM_RADIUS_M = 250.0  # each target roams within ~500m diameter around its spawn
TARGET_SPEED_RANGE = (2.0, 7.0)  # m/s
# Threat defaults (back to moderate)
THREAT_RADAR = RadarParams()  # P_fa=1e-6, n=1.5, t_ref=5s ...
THREAT_WEAPON = WeaponParams(
    weapon_type=WeaponType.MISSILE,
    a_range=3500.0,
    b_slope=2.0,
    omega=0.8,
    t_fire=3.0,
)
THREAT_DEFAULT = AirDefenseThreat(radar=THREAT_RADAR, weapon=THREAT_WEAPON)

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
SOURCE_NAME = "test"
MAX_FUEL_L = 1000.0  # spec max
# 2-hour endurance target -> burn the full tank over ~7200s
LAH_FUEL_BURN_LPS = MAX_FUEL_L / (2 * 3600.0)
DEFAULT_FOV_ASPECT = 16 / 9
_log = lambda msg: print(f"[sim-log] {msg}", flush=True)
SCENARIO_PATH = REPO_ROOT / "mission" / "Scenario_2025-12-14T013406" / "SBC3"


def now_ms_2000() -> int:
    return int((datetime.now(timezone.utc) - _EPOCH_2000).total_seconds() * 1000)


def _datalink_status_for_lah(lah_pos, snapshots, dem: DEM):
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
    # fallback priority even if crashed/absent
    return 4 if len(crashed) >= 4 else 1


def _fuel_warning(fuel_l: float) -> int:
    pct = fuel_l / MAX_FUEL_L if MAX_FUEL_L > 0 else 0.0
    if pct <= 0.01:
        return 3
    if pct <= 0.2:
        return 2
    return 1


def _compute_footprint(uav_pos, tgt_pos, fov_diag_deg, dem: DEM, aspect_ratio=DEFAULT_FOV_ASPECT):
    if tgt_pos is None:
        return [], None, None
    uav_pos = np.array(uav_pos, dtype=float)
    forward = np.array(tgt_pos, dtype=float) - uav_pos
    if np.linalg.norm(forward) < 1e-6:
        return [], None, None
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
    # order: TL, TR, BR, BL
    combos = [(-1, 1), (1, 1), (1, -1), (-1, -1)]
    corners_env = []
    for sx, sy in combos:
        d = forward + sx * tan_h * right + sy * tan_v * up
        d = d / np.linalg.norm(d)
        hit = ray_intersect_dem(uav_pos, d, dem)
        if hit is None:
            return [], None, None
        corners_env.append(hit)

    footprint_lonlat = []
    for c in corners_env:
        lon, lat = dem.env_to_lonlat(c[0], c[1])
        footprint_lonlat.append({"latitude": lat, "longitude": lon, "altitude": float(c[2])})

    tgt_lon, tgt_lat = dem.env_to_lonlat(tgt_pos[0], tgt_pos[1])
    center_coord = {"latitude": tgt_lat, "longitude": tgt_lon, "altitude": float(tgt_pos[2])}
    return footprint_lonlat, center_coord, forward


def _make_agent_state(idx, snap, uav_type, fuel_l, crashed, crippled, snapshots, dem: DEM, targets, fov_diag):
    lon, lat = dem.env_to_lonlat(snap[0], snap[1])
    health = 2 if crashed[idx] or crippled[idx] else 1
    is_unmanned = uav_type != "LAH"
    # pick nearest target for sensor/footprint
    target = None
    if targets:
        target = min(targets, key=lambda T: (T.x - snap[0]) ** 2 + (T.y - snap[1]) ** 2 + (T.z - snap[2]) ** 2)
        target_pos = (target.x, target.y, target.z)
        target_id = getattr(target, "id", None) or targets.index(target) + 1
    else:
        target_pos = None
        target_id = None

    footprint_list, center_coord, _ = _compute_footprint(snap[:3], target_pos, fov_diag, dem)
    sensor_info = {
        "operationalMode": 1,
        "sensorType": 2,
        "fov": float(fov_diag),
        "centerCoordinate": center_coord
        or {"latitude": lat, "longitude": lon, "altitude": int(snap[2])},
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
        fuel_warn = 1 if uav_type == "UAV" else _fuel_warning(fuel_l)
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


def build_agent_status_snapshot(uav_types, snapshots, fuel_levels, crashed, crippled, dem: DEM, targets, fov_diag):
    agent_states = []
    for i, snap in enumerate(snapshots):
        fuel = fuel_levels[i] if i < len(fuel_levels) else 0.0
        agent_states.append(
            _make_agent_state(i, snap, uav_types[i], fuel, crashed, crippled, snapshots, dem, targets, fov_diag)
        )
    return {"timestamp": now_ms_2000(), "source": SOURCE_NAME, "agentStateList": agent_states}


def uav_worker(idx, uav, dem, run_event, crashed_flags, crippled_flags, lock, time_scale, time_lock):
    prev = time.perf_counter()
    while run_event.is_set():
        now = time.perf_counter()
        dt = max(0.001, min(0.05, now - prev))
        prev = now
        with time_lock:
            ts = time_scale["value"]
        dt *= ts
        with lock:
            if not crashed_flags[idx]:
                uav.step(dt)
                # If crippled, force rapid descent and spinning regardless of commands
                if crippled_flags[idx]:
                    uav.s.z = max(0.0, uav.s.z - 30.0 * dt)
                    uav.s.u = max(0.0, uav.s.u * 0.95)
                ground_z = dem.get_height(uav.s.x, uav.s.y)
                if uav.s.z <= ground_z + 0.5:
                    crashed_flags[idx] = True
                    crippled_flags[idx] = False
                    uav.s.z = ground_z
                    uav.s.u = 0.0
                    uav.cmd_throttle = 0.0
                    uav.cmd_pitch_rate = 0.0
                    uav.cmd_yaw_rate = 0.0
                    uav.cmd_roll_rate = 0.0
        time.sleep(0.002)


def main():
    dem = DEM(str(DEM_FILE))
    print(f"[DEM] file: {DEM_FILE}")
    print(
        f"[DEM] shape: {dem.elevation.shape} lon/lat bounds: ({dem.xmin:.5f},{dem.ymin:.5f})-({dem.xmax:.5f},{dem.ymax:.5f})"
    )
    print(f"[DEM] elevation min/max: {dem.min_elev:.1f}/{dem.max_elev:.1f}")

    pygame.init()
    caption = os.getenv("SIM_INSTANCE_NAME", "UAV 6-DoF 3D + DEM + Missile (modular)")
    pygame.display.set_caption(caption)
    print(f"[sim] window caption: {caption}")
    pygame.display.set_mode((WIN_W, WIN_H), DOUBLEBUF | OPENGL)
    clock = pygame.time.Clock()

    # GL info
    print("[GL] vendor:", glGetString(GL_VENDOR))
    print("[GL] renderer:", glGetString(GL_RENDERER))
    print("[GL] version:", glGetString(GL_VERSION))
    print("[GL] GLSL:", glGetString(GL_SHADING_LANGUAGE_VERSION))

    glViewport(0, 0, WIN_W, WIN_H)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glClearColor(*CLEAR_COLOR)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60.0, WIN_W / WIN_H, 0.1, 30000.0)

    glEnable(GL_FOG)
    fog_color = (GLfloat * 4)(*FOG_COLOR)
    glFogfv(GL_FOG_COLOR, fog_color)
    glFogi(GL_FOG_MODE, GL_EXP2)
    glFogf(GL_FOG_DENSITY, FOG_DENSITY)
    glHint(GL_FOG_HINT, GL_NICEST)

    params_uav = UAVParams()
    params_lah = LAHParams()
    uavs = [LAH(params_lah) for _ in range(3)] + [UAV(params_uav) for _ in range(3)]
    uav_types = ["LAH"] * 3 + ["UAV"] * 3
    uav_locks = [threading.Lock() for _ in uavs]

    offsets = [(-60.0, -60.0), (0.0, -60.0), (60.0, -60.0), (-60.0, 60.0), (0.0, 60.0), (60.0, 60.0)]
    for u, (ox, oy) in zip(uavs, offsets):
        u.s.x += ox
        u.s.y += oy
    active_idx = 0
    cam = OrbitCamera()

    targets = [
        MovingTarget(
            x=tx,
            y=ty,
            vmin=TARGET_SPEED_RANGE[0],
            vmax=TARGET_SPEED_RANGE[1],
            roam_center=(tx, ty),
            roam_radius=TARGET_ROAM_RADIUS_M,
            threat=AirDefenseThreat(radar=THREAT_RADAR, weapon=THREAT_WEAPON),
        )
        for tx, ty in TARGET_SPAWNS
    ]
    if not targets:
        targets = [
            MovingTarget(
                x=300.0,
                y=0.0,
                vmin=TARGET_SPEED_RANGE[0],
                vmax=TARGET_SPEED_RANGE[1],
                roam_center=(300.0, 0.0),
                roam_radius=TARGET_ROAM_RADIUS_M,
                threat=AirDefenseThreat(radar=THREAT_RADAR, weapon=THREAT_WEAPON),
            )
        ]
    targets_move = True
    missiles = []
    trails = [[] for _ in uavs]
    crashed = [False for _ in uavs]
    crippled = [False for _ in uavs]
    fuel_levels = [MAX_FUEL_L if t == "LAH" else 0.0 for t in uav_types]
    latest_agent_status_0401 = None

    run_event = threading.Event()
    run_event.set()
    time_scale = {"value": 1.0}
    time_lock = threading.Lock()
    workers = [
        threading.Thread(
            target=uav_worker,
            args=(i, u, dem, run_event, crashed, crippled, uav_locks[i], time_scale, time_lock),
            daemon=True,
        )
        for i, u in enumerate(uavs)
    ]
    for t in workers:
        t.start()

    orbit = False
    pan = False
    last = (0, 0)
    fov_diag = DEFAULT_FOV_DIAG
    fog_enabled = True
    debug_frames = 0

    dt_smoothed = 1 / 60
    running = True
    rotor_angle = 0.0
    _log("entering main loop")
    while running:
        ms = clock.tick(FPS)
        raw_dt = max(0.0001, min(0.05, ms / 1000.0))
        dt_smoothed = 0.85 * dt_smoothed + 0.15 * raw_dt
        dt = dt_smoothed
        with time_lock:
            time_scale_val = time_scale["value"]
        dt_sim = dt * time_scale_val
        rotor_angle = (rotor_angle + 720.0 * dt_sim) % 360.0  # simple spin for visualization

        for i, ttype in enumerate(uav_types):
            if ttype == "LAH" and not crashed[i]:
                fuel_levels[i] = max(0.0, fuel_levels[i] - LAH_FUEL_BURN_LPS * dt_sim)
        if debug_frames < 3:
            _log(f"frame start dt={dt:.4f} dt_sim={dt_sim:.4f} fuel_lah1={fuel_levels[0]:.1f}")

        keys = pygame.key.get_pressed()
        uav = uavs[active_idx]
        is_lah = uav_types[active_idx] == "LAH"
        with uav_locks[active_idx]:
            if not crippled[active_idx]:
                if keys[pygame.K_w]:
                    uav.cmd_throttle = 1.0
                elif keys[pygame.K_s]:
                    uav.cmd_throttle = -1.0
                else:
                    uav.cmd_throttle = 0.0

                if keys[pygame.K_SPACE]:
                    uav.cmd_hover() if is_lah else uav.cmd_straight()
                else:
                    if keys[pygame.K_LEFT]:
                        uav.cmd_left()
                    elif keys[pygame.K_RIGHT]:
                        uav.cmd_right()
                    else:
                        uav.cmd_yaw_rate = 0.0
                        uav.cmd_roll_rate = -uav.s.roll * 2.0
                    if keys[pygame.K_UP]:
                        uav.cmd_climb()
                    elif keys[pygame.K_DOWN]:
                        uav.cmd_descend()
                    else:
                        uav.neutralize_pitch()

        if keys[pygame.K_e]:
            fov_diag += 15.0 * dt
        if keys[pygame.K_d]:
            fov_diag -= 15.0 * dt
        fov_diag = clamp(fov_diag, 1.2, 31.2)
        if debug_frames < 1:
            _log("after input/keys")

        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif e.key == pygame.K_r:
                    for i, u in enumerate(uavs):
                        with uav_locks[i]:
                            u.reset()
                            sp = spawn_points[i] if i < len(spawn_points) else None
                            if sp is not None:
                                u.s.x, u.s.y, u.s.z = sp
                            else:
                                u.s.x += fallback_offsets[i][0]
                                u.s.y += fallback_offsets[i][1]
                            crashed[i] = False
                            crippled[i] = False
                            trails[i].clear()
                            fuel_levels[i] = MAX_FUEL_L if uav_types[i] == "LAH" else 0.0
                elif e.key == pygame.K_f:
                    fog_enabled = not fog_enabled
                elif e.key == pygame.K_n:
                    targets_move = not targets_move
                elif e.key == pygame.K_p:
                    print("[auto] Autopilot not available (manual only)")
                elif e.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    with time_lock:
                        time_scale["value"] = clamp(time_scale["value"] + 1.0, 0.1, 100.0)
                    print(f"[time] scale -> {time_scale['value']:.1f}x")
                elif e.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    with time_lock:
                        time_scale["value"] = clamp(time_scale["value"] - 1.0, 0.1, 100.0)
                    print(f"[time] scale -> {time_scale['value']:.1f}x")
                elif e.key == pygame.K_g:
                    cx, cy, cz = cam.target
                    ox = random.uniform(-150, 150)
                    oy = random.uniform(-150, 150)
                    tx = clamp(cx + ox, -WORLD_HALF, WORLD_HALF)
                    ty = clamp(cy + oy, -WORLD_HALF, WORLD_HALF)
                    targets.append(
                        MovingTarget(
                            x=tx,
                            y=ty,
                            vmin=TARGET_SPEED_RANGE[0],
                            vmax=TARGET_SPEED_RANGE[1],
                            roam_center=(tx, ty),
                            roam_radius=TARGET_ROAM_RADIUS_M,
                            threat=AirDefenseThreat(radar=THREAT_RADAR, weapon=THREAT_WEAPON),
                        )
                    )
                elif e.key == pygame.K_m:
                    if uav_types[active_idx] != "LAH":
                        print("[fire] Missiles available only on LAH (slots 1-3)")
                    elif targets:
                        with uav_locks[active_idx]:
                            nearest = min(
                                targets,
                                key=lambda T: (T.x - uav.s.x) ** 2 + (T.y - uav.s.y) ** 2 + (T.z - uav.s.z) ** 2,
                            )
                            yaw_rad = math.radians(uav.s.yaw)
                            launch_x = uav.s.x + math.cos(yaw_rad) * 8.0
                            launch_y = uav.s.y + math.sin(yaw_rad) * -8.0
                            launch_z = uav.s.z
                        missiles.append(Missile(launch_x, launch_y, launch_z, nearest))
                elif e.key == pygame.K_1:
                    active_idx = 0
                elif e.key == pygame.K_2 and len(uavs) > 1:
                    active_idx = 1
                elif e.key == pygame.K_3 and len(uavs) > 2:
                    active_idx = 2
                elif e.key == pygame.K_4 and len(uavs) > 3:
                    active_idx = 3
                elif e.key == pygame.K_5 and len(uavs) > 4:
                    active_idx = 4
                elif e.key == pygame.K_6 and len(uavs) > 5:
                    active_idx = 5
            elif e.type == pygame.MOUSEBUTTONDOWN:
                if e.button == 3:
                    orbit = True
                    last = e.pos
                elif e.button == 2:
                    pan = True
                    last = e.pos
                elif e.button == 4:
                    cam.distance = max(50.0, cam.distance * 0.9)
                elif e.button == 5:
                    cam.distance = min(8000.0, cam.distance * 1.1)
            elif e.type == pygame.MOUSEBUTTONUP:
                if e.button == 3:
                    orbit = False
                elif e.button == 2:
                    pan = False
            elif e.type == pygame.MOUSEMOTION:
                if orbit:
                    dx = e.pos[0] - last[0]
                    dy = e.pos[1] - last[1]
                    cam.yaw += dx * 0.3
                    cam.pitch = clamp(cam.pitch - dy * 0.3, -89.0, 89.0)
                    last = e.pos
                elif pan:
                    dx = e.pos[0] - last[0]
                    dy = e.pos[1] - last[1]
                    pan_scale = cam.distance * 0.002
                    cam.target[0] -= dx * pan_scale
                    cam.target[1] += dy * pan_scale
                    last = e.pos

        if debug_frames < 1:
            _log("after events loop")

        # Re-evaluate active craft after handling switches
        uav = uavs[active_idx]
        is_lah = uav_types[active_idx] == "LAH"

        snapshots = []
        for i, u in enumerate(uavs):
            with uav_locks[i]:
                s = u.s
                pos = (s.x, s.y, s.z, s.roll, s.pitch, s.yaw, u.s.u)
            snapshots.append(pos)
            # Track trail with spacing and 1 km length limit
            trail = trails[i]
            if trail:
                prev = trail[-1]
                seg = math.dist(pos[:3], prev)
                if seg >= 5.0:
                    trail.append(pos[:3])
            else:
                trail.append(pos[:3])
            total = 0.0
            cutoff_idx = 0
            for j in range(len(trail) - 1, 0, -1):
                total += math.dist(trail[j], trail[j - 1])
                if total > 1000.0:
                    cutoff_idx = j - 1
                    break
            if cutoff_idx > 0:
                trails[i] = trail[cutoff_idx:]
            elif total <= 0.0 and len(trail) > 2:
                trails[i] = trail[-2:]
        active_snap = snapshots[active_idx]
        # Snap camera target to active UAV (no smoothing to reduce overhead)
        cam.target[0] = active_snap[0]
        cam.target[1] = active_snap[1]
        cam.target[2] = active_snap[2]
        if debug_frames < 1:
            _log("after snapshots/trails")

        if targets_move:
            for t in targets:
                t.step(dt_sim, dem=dem)
        for m in list(missiles):
            m.step(dt_sim)
            if (not m.active) and (m.exploded and m.explode_time > 1.2 and len(m.sparks) == 0):
                missiles.remove(m)
        if debug_frames < 1:
            _log("after target/missile step")
        detection_lines = []
        # Threat engagement: targets attack UAVs if detected
        for idx, u in enumerate(uavs):
            with uav_locks[idx]:
                upos = np.array([u.s.x, u.s.y, u.s.z], dtype=float)
            for t in targets:
                if not getattr(t, "threat", None):
                    continue
                range_m = float(np.linalg.norm(np.array([t.x, t.y, t.z]) - upos))
                if range_m < 1e-3:
                    continue
                los = check_los(upos, np.array([t.x, t.y, t.z]), dem)
                t.threat.detection_prob(range_m, los, dt_sim)
                pk = t.threat.kill_prob(range_m, dt_sim)
                detection_lines.append(
                    {
                        "tpos": (t.x, t.y, t.z),
                        "upos": tuple(upos),
                        "los": los,
                        "detected": t.threat.state.detected,
                    }
                )
                if pk > 0.0 and random.random() < pk:
                    with uav_locks[idx]:
                        crippled[idx] = True
                        # Force uncontrolled descent/spin
                        u.cmd_throttle = -1.0
                        u.cmd_pitch_rate = -u.p.max_pitch_rate_dps * 0.8
                        spin_sign = 1.0 if random.random() < 0.5 else -1.0
                        u.cmd_roll_rate = u.p.max_roll_rate_dps * 0.6 * spin_sign
                        u.cmd_yaw_rate = u.p.max_yaw_rate_dps * 0.6 * spin_sign
        if debug_frames < 1:
            _log("after threat eval")
        # Remove destroyed targets
        targets = [t for t in targets if getattr(t, "alive", True)]

        nearest = (
            min(
                targets,
                key=lambda T: (T.x - active_snap[0]) ** 2 + (T.y - active_snap[1]) ** 2 + (T.z - active_snap[2]) ** 2,
            )
            if targets
            else None
        )

        latest_agent_status_0401 = build_agent_status_snapshot(
            uav_types, snapshots, fuel_levels, crashed, crippled, dem, targets, fov_diag
        )
        if debug_frames < 1:
            _log("after agent status build")

        uav_lon, uav_lat = dem.env_to_lonlat(active_snap[0], active_snap[1])

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        cam.apply()

        if fog_enabled:
            glEnable(GL_FOG)
        else:
            glDisable(GL_FOG)

        glDisable(GL_BLEND)
        draw_grid(size=int(RENDER_RADIUS_M), step=100)
        draw_axes(50)
        draw_dem(
            dem,
            cam,
            center_xy=(active_snap[0], active_snap[1]),
            radius_m=RENDER_RADIUS_M,
            prefetch_radius_m=DEM_PREFETCH_RADIUS_M,
            move_threshold=DEM_CACHE_MOVE_THRESHOLD_M,
            z_scale=1.0,
        )
        glEnable(GL_BLEND)

        for idx, trail in enumerate(trails):
            if len(trail) >= 2:
                glDisable(GL_LINE_STIPPLE)
                glLineWidth(2.0 if idx == active_idx else 1.5)
                if idx == active_idx:
                    glColor3f(0.2, 0.6, 1.0)
                else:
                    glColor3f(0.4, 0.5, 0.7)
                glBegin(GL_LINE_STRIP)
                for p in trail:
                    glVertex3f(*p)
                glEnd()

        for i, u in enumerate(uavs):
            with uav_locks[i]:
                if uav_types[i] == "LAH":
                    draw_lah(u, rotor_angle)
                else:
                    draw_uav(u)
        for t in targets:
            t.draw()
        for m in missiles:
            m.draw(dem)
        # Threat LOS lines (targets -> UAVs)
        if detection_lines:
            glLineWidth(1.0)
            glEnable(GL_LINE_STIPPLE)
            glLineStipple(1, 0x0F0F)
            for info in detection_lines:
                if info["los"]:
                    glColor4f(1.0, 0.25, 0.25, 0.5)
                else:
                    glColor4f(0.45, 0.45, 0.45, 0.5)
                glBegin(GL_LINES)
                glVertex3f(*info["tpos"])
                glVertex3f(*info["upos"])
                glEnd()
            glDisable(GL_LINE_STIPPLE)

        # 3D -> 2D labels
        model = glGetDoublev(GL_MODELVIEW_MATRIX)
        proj = glGetDoublev(GL_PROJECTION_MATRIX)
        viewport = glGetIntegerv(GL_VIEWPORT)
        label_entries = []
        for idx, snap in enumerate(snapshots):
            sx, sy, sz = gluProject(snap[0], snap[1], snap[2] + 10.0, model, proj, viewport)
            if 0.0 <= sz <= 1.0:
                color = (255, 80, 80) if idx == active_idx else (220, 220, 220)
                name = f"LAH{idx + 1}" if idx < 3 else f"UAV{idx - 2}"
                speed_text = f"{snap[6]:.0f}m/s"
                state_tag = "FALL" if crippled[idx] else ("CRASH" if crashed[idx] else "")
                text = f"{name} {speed_text}" if not state_tag else f"{name} {speed_text} {state_tag}"
                label_entries.append((sx, sy + 8, text, color))
        for tidx, t in enumerate(targets):
            sx, sy, sz = gluProject(t.x, t.y, t.z + 10.0, model, proj, viewport)
            if 0.0 <= sz <= 1.0:
                label_entries.append((sx, sy + 8, f"target{tidx + 1}", (255, 200, 80)))
        draw_labels(label_entries)

        footprint_area = None  # reset each frame
        if nearest and (not is_lah):
            uav_pos = np.array([active_snap[0], active_snap[1], active_snap[2]], dtype=float)
            tgt_pos = np.array([nearest.x, nearest.y, nearest.z], dtype=float)
            los_clear = check_los(uav_pos, tgt_pos, dem)
            glLineWidth(2.0)
            glEnable(GL_LINE_STIPPLE)
            glLineStipple(1, 0x3333)  # wide-gap dash
            glColor3f(0.6, 0.6, 0.6)
            glBegin(GL_LINES)
            glVertex3f(*uav_pos)
            glVertex3f(*tgt_pos)
            glEnd()
            glDisable(GL_LINE_STIPPLE)
            with uav_locks[active_idx]:
                footprint_area = draw_camera_footprint(
                    uav,
                    (nearest.x, nearest.y, nearest.z),
                    fov_diag_deg=fov_diag,
                    dem=dem,
                    aspect_ratio=16 / 9,
                    z_scale=1.0,
                )
        else:
            los_clear = False

        lines = [
            f"Active: {'LAH' if is_lah else 'UAV'}{active_idx + 1}  pos (m): x={active_snap[0]:7.1f}  y={active_snap[1]:7.1f}  z={active_snap[2]:6.1f}",
            f"pos (lat/lon): lat={uav_lat:9.5f}  lon={uav_lon:10.5f}",
            f"spd (m/s): {active_snap[6]:5.1f}   yaw={active_snap[5]:6.1f}deg  pitch={active_snap[4]:5.1f}deg  roll={active_snap[3]:5.1f}deg",
            f"FOV (diag): {fov_diag:4.1f}deg   LOS: {1 if los_clear else 0}   dt={dt_sim*1000:.1f}ms  time x{time_scale_val:.1f}",
            f"Targets: {len(targets)}  Missiles: {len(missiles)}",
            "1-6: switch craft (1-3 LAH, 4-6 UAV) | M: Fire (LAH only) | N: Toggle targets | G: Spawn target | Wheel: Zoom | RMB drag: Orbit | MMB drag: Pan",
        ]
        if footprint_area is not None:
            lines.append(f"Footprint area: {footprint_area:8.1f} m^2")
        if any(crippled) and not any(crashed):
            crippled_ids = [str(i + 1) for i, c in enumerate(crippled) if c]
            lines.append(f"Status: DAMAGED UAVs: {', '.join(crippled_ids)} (falling)")
        if any(crashed):
            crashed_ids = [str(i + 1) for i, c in enumerate(crashed) if c]
            lines.append(f"Status: CRASHED craft: {', '.join(crashed_ids)} (press R to reset)")
        draw_hud(lines)

        if debug_frames < 3:
            print(
                f"[frame] cam_target=({cam.target[0]:.1f},{cam.target[1]:.1f},{cam.target[2]:.1f}) "
                f"cam_dist={cam.distance:.1f} uav_pos=({active_snap[0]:.1f},{active_snap[1]:.1f},{active_snap[2]:.1f}) "
                f"time_scale={time_scale_val:.1f} fog={fog_enabled}"
            )
            err = glGetError()
            if err:
                print(f"[GL ERROR] frame: {err}")
            debug_frames += 1

        pygame.display.flip()
        if debug_frames < 2:
            _log("after display flip")

    run_event.clear()
    for t in workers:
        t.join(timeout=0.2)
    pygame.quit()


if __name__ == "__main__":
    main()
import traceback
