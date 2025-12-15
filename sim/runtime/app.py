import json
import math
import os
import random
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

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
    glOrtho,
    glPushMatrix,
    glPopMatrix,
    glMatrixMode,
    GL_MODELVIEW,
    GL_PROJECTION,
    GL_QUADS,
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
    GL_LINES,
    GL_LINE_STRIP,
    GL_LINE_LOOP,
    GL_LINE_STIPPLE,
    GL_NICEST,
    GL_POINTS,
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
    glPointSize,
)
from OpenGL.GLU import gluPerspective, gluProject

from sim.agent_status import LAH_FUEL_BURN_LPS, MAX_FUEL_L, build_agent_status_snapshot, now_ms_2000
from sim.config import (
    CLEAR_COLOR,
    DEM_CACHE_MOVE_THRESHOLD_M,
    DEM_FILE,
    DEM_PREFETCH_RADIUS_M,
    FPS,
    FOG_COLOR,
    FOG_DENSITY,
    MAP_DIR,
    REPO_ROOT,
    RENDER_RADIUS_M,
    WIN_H,
    WIN_W,
    WORLD_HALF,
    clamp,
    wrap_deg,
)
from sim.core.camera import OrbitCamera
from sim.entities.entities import Missile, MovingTarget
from sim.entities.threat import AirDefenseThreat
from sim.render.draw import (
    draw_axes,
    draw_camera_footprint,
    draw_dem,
    draw_grid,
    draw_hud,
    draw_labels,
    draw_rejoin_points,
    draw_lah,
    draw_uav,
)
from sim.world.dem import DEM, check_los
from .Std_off import build_standoff_controller, build_standoff_missions
from .UAV_Tracking import TrackingController
from .autopilot import build_missions, update_autopilot
from .constants import INITIAL_OFFSETS, TARGET_ROAM_RADIUS_M, TARGET_SPEED_RANGE, THREAT_RADAR, THREAT_WEAPON, THREAT_DEFAULT
from .state import SimulationState, build_initial_state
from .workers import uav_worker
from . import scenario_loader


class LineScanState:
    def __init__(self, path: list[tuple[float, float, float]], speed: float):
        self.path = path
        self.speed = max(speed, 0.0)
        self.seg_idx = 0
        self.t_along = 0.0
        self.finished = False
        self._cache_last = path[-1] if path else None

    def step(self, dt: float):
        if not self.path or self.finished:
            return self._cache_last
        remaining_dt = dt
        while remaining_dt > 0.0 and not self.finished:
            if self.seg_idx >= len(self.path) - 1:
                self.finished = True
                self.t_along = 0.0
                break
            p0 = np.array(self.path[self.seg_idx], dtype=float)
            p1 = np.array(self.path[self.seg_idx + 1], dtype=float)
            seg_vec = p1 - p0
            seg_len = float(np.linalg.norm(seg_vec))
            if seg_len < 1e-3 or self.speed <= 0.0:
                self.seg_idx += 1
                self.t_along = 0.0
                continue
            seg_dir = seg_vec / seg_len
            dist_left = seg_len - self.t_along
            travel = self.speed * remaining_dt
            if travel < dist_left:
                self.t_along += travel
                remaining_dt = 0.0
            else:
                remaining_dt -= dist_left / self.speed
                self.seg_idx += 1
                self.t_along = 0.0
        if self.seg_idx >= len(self.path) - 1:
            self.finished = True
            return self.path[-1]
        # interpolate along current segment
        p0 = np.array(self.path[self.seg_idx], dtype=float)
        p1 = np.array(self.path[self.seg_idx + 1], dtype=float)
        seg_vec = p1 - p0
        seg_len = float(np.linalg.norm(seg_vec))
        if seg_len < 1e-6:
            return tuple(p0)
        target = p0 + seg_vec * (self.t_along / seg_len)
        return tuple(target)


def _log(msg: str):
    print(f"[sim-log] {msg}", flush=True)


def _env_flag(name: str, default: bool = False) -> bool:
    val = os.getenv(name)
    if val is None:
        return default
    return str(val).strip().lower() in ("1", "true", "yes", "on")


class SimulationApp:
    def __init__(self):
        self.cam = OrbitCamera()
        self.clock: pygame.time.Clock | None = None
        self.dem: DEM | None = None
        self.state: SimulationState | None = None
        self.orbit = False
        self.pan = False
        self.last = (0, 0)
        self.debug_frames = 0
        self.dt_smoothed = 1 / 60
        self.running = True
        self.rotor_angle = 0.0
        self.log_thread: threading.Thread | None = None
        self.log_stop_event: threading.Event | None = None
        self.log_file_path: Path | None = None
        self.log_file_path_0402: Path | None = None
        self.mission_name: str = ""
        self.logged_targets_0402: set[int] = set()
        self.dem_path: Path = DEM_FILE
        self.render_cutoff_m: float = RENDER_RADIUS_M * 1.3  # skip draw/LOS for very distant objects
        self.line_scan_states: list = []
        self._debug_auto_steps: int = 0
        self.auto_mode: str = "manual"  # manual | flightpath | standoff
        self.show_minimap: bool = False
        self.db_root: Path = Path(os.getenv("SIM_DB_ROOT", REPO_ROOT / "database")).expanduser().resolve()
        log_root = Path(os.getenv("SIM_LOG_DIR", REPO_ROOT / "log")).expanduser().resolve()
        self.log_dir_0401: Path = Path(os.getenv("SIM_LOG_0401_DIR", log_root)).expanduser().resolve()
        self.log_dir_0402: Path = Path(os.getenv("SIM_LOG_0402_DIR", log_root)).expanduser().resolve()
        self.headless: bool = _env_flag("SIM_HEADLESS", False)
        self.auto_start: bool = _env_flag("SIM_AUTOSTART", False)
        self.exit_on_mission_done: bool = _env_flag("SIM_EXIT_ON_MISSION_DONE", False)
        exit_after = os.getenv("SIM_EXIT_AFTER_SEC")
        self.auto_exit_after: float | None = None
        if exit_after:
            try:
                self.auto_exit_after = float(exit_after)
            except ValueError:
                self.auto_exit_after = None
        ts_override = os.getenv("SIM_TIME_SCALE")
        self.time_scale_override: float | None = None
        if ts_override:
            try:
                self.time_scale_override = float(ts_override)
            except ValueError:
                self.time_scale_override = None
        self._run_started_at: float | None = None
        self._tick_prev: float | None = None

    def _mission_for_idx(self, idx: int):
        if not self.state or idx >= len(self.state.missions):
            return None
        return self.state.missions[idx]

    def _autopilot_active_for(self, idx: int) -> bool:
        if not self.state:
            return False
        mission = self._mission_for_idx(idx)
        return bool(self.state.autopilot_enabled and mission and mission.active and not mission.completed)

    def _autopilot_finished(self) -> bool:
        """Return True when all programmed missions (or standoff) are done."""
        if not self.state or not getattr(self.state, "autopilot_enabled", False):
            return False
        missions = [m for m in getattr(self.state, "missions", []) if m]
        ctrl = getattr(self.state, "standoff_controller", None)
        standoff_done = bool(ctrl) and bool(getattr(ctrl, "mission_done", False))
        missions_done = bool(missions) and all(getattr(m, "completed", False) for m in missions)
        crashed_out = all(getattr(self.state, "crashed", [])) if getattr(self.state, "crashed", None) else False
        return missions_done or standoff_done or crashed_out

    def setup(self):
        # Pick DEM tile based on mission reference coords (fallback to default)
        coords_hint = scenario_loader.peek_reference_coords(self.db_root)
        dem_candidate = scenario_loader.pick_dem_for_coords(coords_hint, MAP_DIR)
        if dem_candidate:
            self.dem_path = dem_candidate
        self.dem = DEM(str(self.dem_path))
        self._log_dem_info()
        if not self.headless:
            self._init_display()
        self.state = build_initial_state()
        scenario = scenario_loader.load_scenario(self.db_root, self.dem)
        if scenario:
            scenario_loader.apply_scenario_to_state(self.state, scenario, self.dem)
        if self.time_scale_override is not None:
            with self.state.time_lock:
                self.state.time_scale["value"] = clamp(self.time_scale_override, 0.1, 100.0)
        self._start_workers()
        self._start_logger()
        if self.auto_start:
            print("[auto] SIM_AUTOSTART enabled; starting autopilot missions.")
            self._start_autopilot_missions()
        _log("entering main loop")

    def run(self):
        self.setup()
        self._run_started_at = time.time()
        try:
            while self.running:
                dt, dt_sim, time_scale_val = self._tick_time()
                self._consume_fuel(dt_sim)
                if not self.headless:
                    keys = pygame.key.get_pressed()
                    self._apply_continuous_input(keys, dt)
                    self._process_events()
                self._update_autopilot(dt_sim)
                snapshots, active_snap = self._capture_snapshots_and_trails()
                nearest = self._nearest_target(active_snap)
                if self.auto_mode == "flightpath":
                    nearest = None
                self._update_targets_and_missiles(dt_sim)
                detection_lines = self._evaluate_threats(dt_sim)
                self._remove_destroyed_targets()
                self.state.latest_agent_status_0401 = build_agent_status_snapshot(
                    self.state.uav_types,
                    snapshots,
                    self.state.fuel_levels,
                    self.state.crashed,
                    self.state.crippled,
                    self.dem,
                    self.state.targets,
                    self.state.fov_diag,
                )
                uav_lon, uav_lat = self.dem.env_to_lonlat(active_snap[0], active_snap[1])
                if self.headless:
                    self._headless_detection_and_tracking(snapshots, active_snap, nearest)
                else:
                    self._render_frame(
                        snapshots, active_snap, nearest, detection_lines, dt_sim, time_scale_val, uav_lon, uav_lat
                    )
                if self.auto_exit_after is not None and self._run_started_at is not None:
                    if time.time() - self._run_started_at >= self.auto_exit_after:
                        print(f"[auto] exit after {self.auto_exit_after:.1f}s (SIM_EXIT_AFTER_SEC).")
                        self.running = False
                        continue
                if self.exit_on_mission_done and self._autopilot_finished():
                    print("[auto] exit: missions completed (SIM_EXIT_ON_MISSION_DONE).")
                    self.running = False
                    continue
        finally:
            self._shutdown()

    def _log_dem_info(self):
        assert self.dem is not None
        print(f"[DEM] file: {self.dem_path}")
        print(
            f"[DEM] shape: {self.dem.elevation.shape} lon/lat bounds: ({self.dem.xmin:.5f},{self.dem.ymin:.5f})-({self.dem.xmax:.5f},{self.dem.ymax:.5f})"
        )
        print(f"[DEM] elevation min/max: {self.dem.min_elev:.1f}/{self.dem.max_elev:.1f}")

    def _init_display(self):
        pygame.init()
        caption = os.getenv("SIM_INSTANCE_NAME", "UAV 6-DoF 3D + DEM + Missile (modular)")
        pygame.display.set_caption(caption)
        print(f"[sim] window caption: {caption}")
        pygame.display.set_mode((WIN_W, WIN_H), DOUBLEBUF | OPENGL)
        self.clock = pygame.time.Clock()

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

    def _start_workers(self):
        assert self.dem is not None
        assert self.state is not None
        self.state.workers = [
            threading.Thread(
                target=uav_worker,
                args=(
                    i,
                    u,
                    self.dem,
                    self.state.run_event,
                    self.state.crashed,
                    self.state.crippled,
                    self.state.uav_locks[i],
                    self.state.time_scale,
                    self.state.time_lock,
                ),
                daemon=True,
            )
            for i, u in enumerate(self.state.uavs)
        ]
        for t in self.state.workers:
            t.start()

    def _start_logger(self):
        log_dir_0401 = self.log_dir_0401
        log_dir_0402 = self.log_dir_0402
        log_dir_0401.mkdir(parents=True, exist_ok=True)
        log_dir_0402.mkdir(parents=True, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%f")[:-3]  # UTC, ms precision
        pid = os.getpid()
        base = os.getenv("SIM_MISSION_NAME", f"{ts}_{pid}")
        self.mission_name = base
        self.log_file_path = log_dir_0401 / f"0401_{base}.njson"
        self.log_file_path_0402 = log_dir_0402 / f"0402_{base}.njson"
        self.log_stop_event = threading.Event()

        def _loop():
            while not self.log_stop_event.is_set():
                if self.state and self.state.latest_agent_status_0401:
                    try:
                        with self.log_file_path.open("a", encoding="utf-8") as f:
                            f.write(json.dumps(self.state.latest_agent_status_0401, ensure_ascii=False) + "\n")
                    except Exception as e:
                        print(f"[log] write failed: {e}")
                time.sleep(0.2)  # 5 Hz

        self.log_thread = threading.Thread(target=_loop, daemon=True)
        self.log_thread.start()

    def _log_0402_detection(self, aircraft_id: int, target_idx: int, target):
        if self.dem is None or self.log_file_path_0402 is None:
            return
        key = id(target)
        if key in self.logged_targets_0402:
            return
        self.logged_targets_0402.add(key)
        try:
            lon, lat = self.dem.env_to_lonlat(target.x, target.y)
            alt = float(getattr(target, "z", 0.0))
            target_type = getattr(target, "targetType", None) or getattr(target, "type", None) or 1
            ts_0401 = None
            if self.state and getattr(self.state, "latest_agent_status_0401", None):
                ts_0401 = self.state.latest_agent_status_0401.get("timestamp")
            ts = ts_0401 if ts_0401 is not None else now_ms_2000()
            payload = {
                "timestamp": ts,
                "roiInfo": {
                    "aircraftID": int(aircraft_id),
                    "coordinate": {"latitude": lat, "longitude": lon, "altitude": alt},
                    "fov": float(self.state.fov_diag if self.state else 0.0),
                },
                "targetList": [
                    {
                        "targetID": int(target_idx),
                        "targetType": int(target_type),
                        "coordinate": {"latitude": lat, "longitude": lon, "altitude": alt},
                        "watcher": {"aircraftID": int(aircraft_id)},
                        "targetInFrame": 1,
                        "isDestroyed": 0,
                        "threat": 100.0,
                    }
                ],
            }
            with self.log_file_path_0402.open("a", encoding="utf-8") as f:
                f.write(json.dumps(payload, ensure_ascii=False) + "\n")
        except Exception as e:
            print(f"[log0402] write failed: {e}")

    def _tick_time(self):
        if self.headless or self.clock is None:
            now = time.perf_counter()
            if self._tick_prev is None:
                self._tick_prev = now
            raw_dt = now - self._tick_prev
            target_dt = 1.0 / FPS
            if raw_dt < target_dt:
                time.sleep(target_dt - raw_dt)
                now = time.perf_counter()
                raw_dt = now - self._tick_prev
            self._tick_prev = now
        else:
            ms = self.clock.tick(FPS)
            raw_dt = ms / 1000.0
        raw_dt = max(0.0001, min(0.05, raw_dt))
        self.dt_smoothed = 0.85 * self.dt_smoothed + 0.15 * raw_dt
        dt = self.dt_smoothed
        with self.state.time_lock:
            time_scale_val = self.state.time_scale["value"]
        dt_sim = dt * time_scale_val
        self.rotor_angle = (self.rotor_angle + 720.0 * dt_sim) % 360.0
        if self.debug_frames < 3:
            _log(f"frame start dt={dt:.4f} dt_sim={dt_sim:.4f} fuel_lah1={self.state.fuel_levels[0]:.1f}")
        return dt, dt_sim, time_scale_val

    def _consume_fuel(self, dt_sim):
        for i, _ in enumerate(self.state.uav_types):
            if not self.state.crashed[i]:
                self.state.fuel_levels[i] = max(0.0, self.state.fuel_levels[i] - LAH_FUEL_BURN_LPS * dt_sim)

    def _apply_continuous_input(self, keys, dt):
        active_idx = self.state.active_idx
        uav = self.state.uavs[active_idx]
        is_lah = self.state.uav_types[active_idx] == "LAH"
        autop_active = self._autopilot_active_for(active_idx)
        if not autop_active:
            with self.state.uav_locks[active_idx]:
                if not self.state.crippled[active_idx]:
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
            self.state.fov_diag += 15.0 * dt
        if keys[pygame.K_d]:
            self.state.fov_diag -= 15.0 * dt
        self.state.fov_diag = clamp(self.state.fov_diag, 1.2, 31.2)
        if self.debug_frames < 1:
            _log("after input/keys")

    def _process_events(self):
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                self.running = False
            elif e.type == pygame.KEYDOWN:
                self._handle_keydown(e.key)
            elif e.type == pygame.MOUSEBUTTONDOWN:
                self._handle_mouse_down(e)
            elif e.type == pygame.MOUSEBUTTONUP:
                self._handle_mouse_up(e)
            elif e.type == pygame.MOUSEMOTION:
                self._handle_mouse_motion(e)
        if self.debug_frames < 1:
            _log("after events loop")

    def _handle_keydown(self, key):
        if key in (pygame.K_ESCAPE, pygame.K_q):
            self.running = False
        elif key == pygame.K_r:
            self._reset_all_craft(stop_autopilot=True)
        elif key == pygame.K_f:
            self.state.fog_enabled = not self.state.fog_enabled
        elif key == pygame.K_n:
            self.state.targets_move = not self.state.targets_move
        elif key == pygame.K_p:
            self._start_autopilot_missions()
        elif key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            with self.state.time_lock:
                self.state.time_scale["value"] = clamp(self.state.time_scale["value"] + 1.0, 0.1, 100.0)
            print(f"[time] scale -> {self.state.time_scale['value']:.1f}x")
        elif key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            with self.state.time_lock:
                self.state.time_scale["value"] = clamp(self.state.time_scale["value"] - 1.0, 0.1, 100.0)
            print(f"[time] scale -> {self.state.time_scale['value']:.1f}x")
        elif key == pygame.K_g:
            self._spawn_target_near_camera()
        elif key == pygame.K_m:
            self._fire_missile()
        elif key == pygame.K_o:
            self.state.threat_kill_disabled = True
            print("[threat] attacks disabled (kill switch 'O').")
        elif key == pygame.K_F1:
            self.show_minimap = not self.show_minimap
            print(f"[minimap] {'ON' if self.show_minimap else 'OFF'}")
        elif key == pygame.K_1:
            self.state.active_idx = 0
        elif key == pygame.K_2 and len(self.state.uavs) > 1:
            self.state.active_idx = 1
        elif key == pygame.K_3 and len(self.state.uavs) > 2:
            self.state.active_idx = 2
        elif key == pygame.K_4 and len(self.state.uavs) > 3:
            self.state.active_idx = 3
        elif key == pygame.K_5 and len(self.state.uavs) > 4:
            self.state.active_idx = 4
        elif key == pygame.K_6 and len(self.state.uavs) > 5:
            self.state.active_idx = 5

    def _handle_mouse_down(self, event):
        if event.button == 3:
            self.orbit = True
            self.last = event.pos
        elif event.button == 2:
            self.pan = True
            self.last = event.pos
        elif event.button == 4:
            self.cam.distance = max(50.0, self.cam.distance * 0.9)
        elif event.button == 5:
            self.cam.distance = min(8000.0, self.cam.distance * 1.1)

    def _handle_mouse_up(self, event):
        if event.button == 3:
            self.orbit = False
        elif event.button == 2:
            self.pan = False

    def _handle_mouse_motion(self, event):
        if self.orbit:
            dx = event.pos[0] - self.last[0]
            dy = event.pos[1] - self.last[1]
            self.cam.yaw += dx * 0.3
            self.cam.pitch = clamp(self.cam.pitch - dy * 0.3, -89.0, 89.0)
            self.last = event.pos
        elif self.pan:
            dx = event.pos[0] - self.last[0]
            dy = event.pos[1] - self.last[1]
            pan_scale = self.cam.distance * 0.002
            self.cam.target[0] -= dx * pan_scale
            self.cam.target[1] += dy * pan_scale
            self.last = event.pos

    def _reset_all_craft(self, stop_autopilot: bool = False, ground_clearance_m: float | None = None):
        if stop_autopilot:
            self.state.autopilot_enabled = False
            self.state.missions = []
            self.state.standoff_controller = None
            self.state.standoff_gimbal_target = None
            self.state.standoff_uav_idx = None
            self.state.lah_qrf_active = False
            self.state.lah_qrf_rtb = False
            self.state.lah_qrf_timer = 0.0
            self.state.lah_qrf_target_idx = None
            self.state.lah_qrf_controller = None
        for i, u in enumerate(self.state.uavs):
            with self.state.uav_locks[i]:
                u.reset()
                sp = self.state.initial_spawn_points[i] if i < len(self.state.initial_spawn_points) else None
                if sp is not None:
                    u.s.x, u.s.y = sp[0], sp[1]
                    spawn_z = sp[2] if len(sp) >= 3 else u.s.z
                else:
                    u.s.x += INITIAL_OFFSETS[i][0]
                    u.s.y += INITIAL_OFFSETS[i][1]
                    spawn_z = u.s.z
                if ground_clearance_m is not None and self.dem is not None:
                    ground_z = self.dem.get_height(u.s.x, u.s.y)
                    u.s.z = ground_z + ground_clearance_m
                else:
                    u.s.z = spawn_z
                self.state.crashed[i] = False
                self.state.crippled[i] = False
                self.state.trails[i].clear()
                self.state.fuel_levels[i] = MAX_FUEL_L

    def _start_autopilot_missions(self):
        if not self.dem:
            print("[auto] DEM not loaded; cannot start autopilot missions.")
            return
        # Reset crafts but keep targets; use ground clearance for safety.
        self._reset_all_craft(stop_autopilot=False, ground_clearance_m=50.0)

        # Try loading predefined flight paths from database/FlightPath.
        missions_by_aid = scenario_loader.load_flightpaths(self.db_root, self.dem)
        if missions_by_aid:
            self.state.missions = [None] * len(self.state.uavs)
            self.state.line_scan_states = [None] * len(self.state.uavs)
            self.state.gimbal_targets = [None] * len(self.state.uavs)
            for idx in range(len(self.state.uavs)):
                aid = idx + 1
                m = missions_by_aid.get(aid)
                if m:
                    self.state.missions[idx] = m
            self.state.autopilot_enabled = True
            self.state.standoff_controller = None
            self.state.standoff_gimbal_target = None
            self.state.standoff_uav_idx = None
            self.state.tracking_controller = None
            self.state.tracking_target_idx = None
            self.state.tracking_active = False
            self.state.tracking_resume_timer = 0.0
            self.state.tracking_cooldown = 0.0
            self.state.standoff_paused = False
            loaded_counts = {aid: len(m.waypoints) for aid, m in missions_by_aid.items()}
            print(f"[auto] Loaded {len(missions_by_aid)} flight paths from database/FlightPath (autopilot on). details={loaded_counts}")
            missing = [idx + 1 for idx, m in enumerate(self.state.missions) if m is None]
            if missing:
                print(f"[auto] No mission for aircraft: {missing} (they will idle).")
            # Debug: show distance to first waypoint for each mission
            for idx, mission in enumerate(self.state.missions):
                if mission and mission.waypoints:
                    wp0 = mission.waypoints[0]
                    u = self.state.uavs[idx]
                    dist = math.hypot(wp0.x - u.s.x, wp0.y - u.s.y)
                    print(f"[auto] craft{idx+1} first wp at ({wp0.x:.1f},{wp0.y:.1f},{wp0.z:.1f}), dist2D={dist:.1f}m")
            self._debug_auto_steps = 10
            self.auto_mode = "flightpath"
            return

        # Fallback to standoff mission if no flight path found.
        self._reset_all_craft(stop_autopilot=False, ground_clearance_m=50.0)
        # Ensure threats are disabled when entering standoff mode (safety default).
        self.state.threat_kill_disabled = True
        self.state.targets = []  # clear old targets so spawned ones are obvious
        self.state.standoff_controller = None
        self.state.standoff_gimbal_target = None
        self.state.standoff_uav_idx = None
        self.state.tracking_controller = None
        self.state.tracking_target_idx = None
        self.state.tracking_active = False

        # Prefer the fixed standoff mission (waypoints + scan sweep)
        standoff_idx = next((i for i, t in enumerate(self.state.uav_types) if t != "LAH"), 0)
        controller = build_standoff_controller(self.state.uavs[standoff_idx], self.dem)
        if controller:
            self.state.standoff_controller = controller
            self.state.standoff_uav_idx = standoff_idx
            self.state.standoff_gimbal_target = None
            self.state.missions = []  # disable random missions
            first_pkg = controller.mission_plan[0] if controller.mission_plan else None
            if first_pkg:
                first_wp = first_pkg.waypoint
                with self.state.uav_locks[standoff_idx]:
                    u = self.state.uavs[standoff_idx]
                    u.s.x, u.s.y, u.s.z = first_wp
                    if len(controller.mission_plan) >= 2:
                        nx, ny, _ = controller.mission_plan[1].waypoint
                        dx = nx - first_wp[0]
                        dy = ny - first_wp[1]
                        u.s.yaw = wrap_deg(math.degrees(math.atan2(-dy, dx)))
            self.state.active_idx = standoff_idx
            self.state.autopilot_enabled = True

            # Position a QRF LAH on the ground beneath the standoff start point.
            lah_idx = 0  # first LAH
            if first_pkg and lah_idx < len(self.state.uavs) and self.state.uav_types[lah_idx] == "LAH":
                ground_z = self.dem.get_height(first_pkg.waypoint[0], first_pkg.waypoint[1])
                base_z = ground_z + 2.0
                with self.state.uav_locks[lah_idx]:
                    lah = self.state.uavs[lah_idx]
                    lah.s.x = first_pkg.waypoint[0]
                    lah.s.y = first_pkg.waypoint[1]
                    lah.s.z = base_z
                    lah.s.yaw = getattr(lah.s, "yaw", 0.0)
                self.state.lah_qrf_idx = lah_idx
                self.state.lah_qrf_origin = (first_pkg.waypoint[0], first_pkg.waypoint[1], base_z)
                self.state.lah_qrf_active = False
                self.state.lah_qrf_rtb = False
                self.state.lah_qrf_timer = 0.0
                self.state.lah_qrf_target_idx = None
                self.state.lah_qrf_controller = None
                # Keep reset positions in sync so future resets place LAH at the pad.
                if lah_idx < len(self.state.initial_spawn_points):
                    self.state.initial_spawn_points[lah_idx] = (first_pkg.waypoint[0], first_pkg.waypoint[1], base_z)

            # Use DEM bounds to avoid clamping targets far away from the route
            xmin, xmax, ymin, ymax = self.dem.get_world_env_bounds()
            target_world_half = max(abs(xmin), abs(xmax), abs(ymin), abs(ymax)) + 500.0

            # Spawn a dedicated trackable target between the provided coordinates
            tx = (19379.7 + 18729.9) * 0.5
            ty = (13010.3 + 13703.4) * 0.5
            track_target = MovingTarget(
                x=tx,
                y=ty,
                vmin=3.0,
                vmax=7.0,
                world_half=target_world_half,
                roam_center=(tx, ty),
                roam_radius=150.0,
                threat=None,  # no threat; tracking-only target
            )
            track_target.trackable = True
            self.state.targets.append(track_target)

            print(f"[auto] Standoff mission loaded ({len(controller.mission_plan)} waypoints) for UAV{standoff_idx + 1}")
            print(f"[auto] Trackable target spawned at ({tx:.1f}, {ty:.1f})")
            self.auto_mode = "standoff"
            return

        # Fallback: build procedural missions for all craft
        origins = [(u.s.x, u.s.y, u.s.z) for u in self.state.uavs]
        self.state.missions = build_missions(origins, self.dem)
        self.state.autopilot_enabled = True
        self.auto_mode = "standoff"
        print(f"[auto] Autopilot started for {len(self.state.missions)} craft (5 waypoints each, +50m AGL)")

    def _spawn_target_near_camera(self):
        cx, cy, cz = self.cam.target
        ox = random.uniform(-150, 150)
        oy = random.uniform(-150, 150)
        tx = clamp(cx + ox, -WORLD_HALF, WORLD_HALF)
        ty = clamp(cy + oy, -WORLD_HALF, WORLD_HALF)
        self.state.targets.append(
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

    def _fire_missile(self):
        active_idx = self.state.active_idx
        if self.state.uav_types[active_idx] != "LAH":
            print("[fire] Missiles available only on LAH (slots 1-3)")
            return
        if not self.state.targets:
            return
        uav = self.state.uavs[active_idx]
        with self.state.uav_locks[active_idx]:
            nearest = min(
                self.state.targets,
                key=lambda T: (T.x - uav.s.x) ** 2 + (T.y - uav.s.y) ** 2 + (T.z - uav.s.z) ** 2,
            )
            yaw_rad = math.radians(uav.s.yaw)
            launch_x = uav.s.x + math.cos(yaw_rad) * 8.0
            launch_y = uav.s.y + math.sin(yaw_rad) * -8.0
            launch_z = uav.s.z
        self.state.missiles.append(Missile(launch_x, launch_y, launch_z, nearest))

    def _capture_snapshots_and_trails(self):
        snapshots = []
        for i, u in enumerate(self.state.uavs):
            with self.state.uav_locks[i]:
                s = u.s
                pos = (s.x, s.y, s.z, s.roll, s.pitch, s.yaw, u.s.u)
            snapshots.append(pos)
            trail = self.state.trails[i]
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
                self.state.trails[i] = trail[cutoff_idx:]
            elif total <= 0.0 and len(trail) > 2:
                self.state.trails[i] = trail[-2:]
        active_snap = snapshots[self.state.active_idx]
        self.cam.target[0] = active_snap[0]
        self.cam.target[1] = active_snap[1]
        self.cam.target[2] = active_snap[2]
        if self.debug_frames < 1:
            _log("after snapshots/trails")
        return snapshots, active_snap

    def _nearest_target(self, active_snap):
        if not self.state.targets:
            return None
        return min(
            self.state.targets,
            key=lambda T: (T.x - active_snap[0]) ** 2 + (T.y - active_snap[1]) ** 2 + (T.z - active_snap[2]) ** 2,
        )

    def _draw_minimap(self, snapshots):
        if not self.dem:
            return
        try:
            # Collect points (missions, UAVs, targets) for local bounding box
            pts = []
            for m in getattr(self.state, "missions", []) or []:
                if not m or not m.waypoints:
                    continue
                pts.extend([(wp.x, wp.y) for wp in m.waypoints])
            pts.extend([(s[0], s[1]) for s in snapshots])
            pts.extend([(t.x, t.y) for t in getattr(self.state, "targets", [])])
            if pts:
                xs, ys = zip(*pts)
                xmin, xmax = min(xs), max(xs)
                ymin, ymax = min(ys), max(ys)
                dx = xmax - xmin
                dy = ymax - ymin
                pad = max(100.0, max(dx, dy) * 0.2)
                xmin -= pad
                xmax += pad
                ymin -= pad
                ymax += pad
            else:
                xmin, xmax, ymin, ymax = self.dem.get_world_env_bounds()
            w, h = 260, 260
            margin = 12
            x0 = WIN_W - w - margin
            y0 = WIN_H - h - margin

            def _map(x, y):
                u = (x - xmin) / max(1e-6, (xmax - xmin))
                v = (y - ymin) / max(1e-6, (ymax - ymin))
                return x0 + u * w, y0 + v * h

            glMatrixMode(GL_PROJECTION)
            glPushMatrix()
            glLoadIdentity()
            glOrtho(0, WIN_W, 0, WIN_H, -1, 1)
            glMatrixMode(GL_MODELVIEW)
            glPushMatrix()
            glLoadIdentity()
            glDisable(GL_DEPTH_TEST)
            glEnable(GL_BLEND)
            # Background
            glColor4f(0.05, 0.1, 0.15, 0.7)
            glBegin(GL_QUADS)
            glVertex3f(x0, y0, 0)
            glVertex3f(x0 + w, y0, 0)
            glVertex3f(x0 + w, y0 + h, 0)
            glVertex3f(x0, y0 + h, 0)
            glEnd()
            # Border
            glColor3f(0.2, 0.8, 0.9)
            glBegin(GL_LINE_LOOP)
            glVertex3f(x0, y0, 0)
            glVertex3f(x0 + w, y0, 0)
            glVertex3f(x0 + w, y0 + h, 0)
            glVertex3f(x0, y0 + h, 0)
            glEnd()
            # Missions (points only)
            glColor3f(0.95, 0.85, 0.2)
            glPointSize(2.5)
            glBegin(GL_POINTS)
            for m in getattr(self.state, "missions", []) or []:
                if not m or not m.waypoints:
                    continue
                for wp in m.waypoints:
                    px, py = _map(wp.x, wp.y)
                    glVertex3f(px, py, 0)
            glEnd()
            # Targets (red)
            glColor3f(1.0, 0.25, 0.25)
            glPointSize(4.0)
            glBegin(GL_POINTS)
            for t in getattr(self.state, "targets", []) or []:
                px, py = _map(t.x, t.y)
                glVertex3f(px, py, 0)
            glEnd()
            # UAVs (white) / LAH (blue)
            glPointSize(5.5)
            glBegin(GL_POINTS)
            for idx, snap in enumerate(snapshots):
                px, py = _map(snap[0], snap[1])
                is_lah = self.state.uav_types[idx] == "LAH"
                if is_lah:
                    glColor3f(0.2, 0.6, 1.0)
                else:
                    glColor3f(0.9, 0.9, 0.95)
                glVertex3f(px, py, 0)
            glEnd()
            glDisable(GL_BLEND)
            glEnable(GL_DEPTH_TEST)
            glPopMatrix()
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
        except Exception as e:
            print(f"[minimap] draw failed: {e}")
    @staticmethod
    def _point_in_poly(px: float, py: float, poly):
        inside = False
        n = len(poly)
        j = n - 1
        for i in range(n):
            xi, yi = poly[i][0], poly[i][1]
            xj, yj = poly[j][0], poly[j][1]
            intersect = ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi + 1e-9) + xi)
            if intersect:
                inside = not inside
            j = i
        return inside

    def _start_tracking(self, target_idx: int, uav_idx: int):
        self.state.tracking_controller = TrackingController(target_idx=target_idx)
        self.state.tracking_target_idx = target_idx
        self.state.tracking_active = True
        self.state.tracking_resume_timer = 10.0
        self.state.tracking_cooldown = 0.0
        self.state.standoff_paused = True
        self._launch_lah_attack(target_idx)
        # Save standoff progress to resume later from same segment/scan progress.
        if self.state.standoff_controller:
            ctrl = self.state.standoff_controller
            self.state.standoff_resume_index = max(0, min(ctrl.mission_index, len(ctrl.mission_plan) - 1))
            if ctrl.scan_duration > 1e-6:
                self.state.standoff_resume_scan_t = clamp(ctrl.scan_timer / ctrl.scan_duration, 0.0, 1.0)
            else:
                self.state.standoff_resume_scan_t = 0.0
        self.state.active_idx = uav_idx
        print(f"[tracking] target#{target_idx + 1} locked by UAV{uav_idx + 1}")

    def _launch_lah_attack(self, target_idx: int):
        if self.state.lah_qrf_idx is None or target_idx < 0 or target_idx >= len(self.state.targets):
            return
        self.state.lah_qrf_active = True
        self.state.lah_qrf_rtb = False
        self.state.lah_qrf_timer = 10.0
        self.state.lah_qrf_target_idx = target_idx
        self.state.lah_qrf_controller = TrackingController(target_idx=target_idx, orbit_radius=120.0, target_speed=60.0)
        print(f"[LAH] QRF launched on target#{target_idx + 1}")

    def _update_lah_qrf(self, dt_sim: float):
        if self.state.lah_qrf_idx is None:
            return
        lah_idx = self.state.lah_qrf_idx
        lah = self.state.uavs[lah_idx]

        if self.state.lah_qrf_active:
            tgt = None
            if self.state.lah_qrf_target_idx is not None and 0 <= self.state.lah_qrf_target_idx < len(self.state.targets):
                tgt = self.state.targets[self.state.lah_qrf_target_idx]
            keep = False
            if self.state.lah_qrf_controller:
                with self.state.uav_locks[lah_idx]:
                    keep = self.state.lah_qrf_controller.update(lah, tgt, self.dem, dt_sim)
            if tgt is not None and getattr(tgt, "alive", True):
                dist = math.hypot(tgt.x - lah.s.x, tgt.y - lah.s.y)
                if dist <= 30.0:
                    tgt.alive = False
                    tgt.color = (0.0, 0.0, 0.0)
                    keep = False
                    print(f"[LAH] target#{self.state.lah_qrf_target_idx + 1} neutralized.")
            self.state.lah_qrf_timer = max(0.0, self.state.lah_qrf_timer - dt_sim)
            if (not keep) or self.state.lah_qrf_timer <= 0.0:
                self.state.lah_qrf_active = False
                self.state.lah_qrf_controller = None
                self.state.lah_qrf_target_idx = None
                self.state.lah_qrf_rtb = True
                if tgt is not None and not getattr(tgt, "alive", True):
                    try:
                        self.state.targets.remove(tgt)
                    except ValueError:
                        pass
                print("[LAH] returning to base.")

        if self.state.lah_qrf_rtb and self.state.lah_qrf_origin is not None:
            ox, oy, oz = self.state.lah_qrf_origin
            with self.state.uav_locks[lah_idx]:
                dx = ox - lah.s.x
                dy = oy - lah.s.y
                dz = oz - lah.s.z
                dist_2d = math.hypot(dx, dy)
                desired_yaw = wrap_deg(math.degrees(math.atan2(-dy, dx))) if dist_2d > 1e-3 else lah.s.yaw
                yaw_err = wrap_deg(desired_yaw - lah.s.yaw)
                lah.cmd_yaw_rate = clamp(yaw_err * 1.2, -lah.p.max_yaw_rate_dps, lah.p.max_yaw_rate_dps)

                # Level out and control altitude gently
                pitch_cmd = clamp(dz * 0.05, -lah.p.max_pitch_rate_dps * 0.5, lah.p.max_pitch_rate_dps * 0.5)
                lah.cmd_pitch_rate = pitch_cmd
                lah.cmd_roll_rate = -lah.s.roll * 1.5

                target_speed = 50.0 if dist_2d > 80.0 else 30.0
                speed_err = target_speed - lah.s.u
                throttle_k = 1.0 / max(lah.p.accel, 1e-3)
                lah.cmd_throttle = clamp(speed_err * throttle_k, -1.0, 1.0)

                if dist_2d < 20.0 and abs(dz) < 20.0:
                    self.state.lah_qrf_rtb = False
                    lah.cmd_hover()

    def _update_autopilot(self, dt_sim):
        if not self.state or not self.dem:
            return
        # Cooldown to prevent immediate re-trigger after resuming standoff.
        if getattr(self.state, "tracking_cooldown", 0.0) > 0.0:
            self.state.tracking_cooldown = max(0.0, self.state.tracking_cooldown - dt_sim)
        if (
            self.state.standoff_controller
            and self.state.standoff_uav_idx is not None
            and not getattr(self.state, "standoff_paused", False)
        ):
            idx = self.state.standoff_uav_idx
            with self.state.uav_locks[idx]:
                tgt = self.state.standoff_controller.update(dt_sim)
            # Default gimbal target follows standoff scan unless tracking overrides below.
            self.state.standoff_gimbal_target = tuple(tgt) if tgt is not None else None
        if self.state.tracking_active and self.state.tracking_controller and self.state.standoff_uav_idx is not None:
            uav_idx = self.state.standoff_uav_idx
            tgt_idx = self.state.tracking_controller.target_idx
            target = self.state.targets[tgt_idx] if 0 <= tgt_idx < len(self.state.targets) else None
            with self.state.uav_locks[uav_idx]:
                keep = self.state.tracking_controller.update(self.state.uavs[uav_idx], target, self.dem, dt_sim)
                # Force the gimbal to stay centered on the target during tracking.
                if target is not None:
                    self.state.standoff_gimbal_target = (target.x, target.y, target.z)
            # Countdown to resume standoff mission
            self.state.tracking_resume_timer = max(0.0, self.state.tracking_resume_timer - dt_sim)
            if (not keep) or self.state.tracking_resume_timer <= 0.0:
                if not keep:
                    print("[tracking] ended (target lost).")
                else:
                    print("[tracking] resume standoff mission after 10s pause.")
                # Remove the tracked target so it cannot immediately retrigger.
                if self.state.tracking_target_idx is not None and 0 <= self.state.tracking_target_idx < len(self.state.targets):
                    del self.state.targets[self.state.tracking_target_idx]
                self.state.tracking_active = False
                self.state.tracking_controller = None
                self.state.tracking_target_idx = None
                self.state.tracking_resume_timer = 0.0
                self.state.tracking_cooldown = 3.0  # prevent instant retrigger if target still in FOV
                self.state.standoff_paused = False
                # Resume standoff exactly where paused; steer back and align entry heading to outbound leg.
                if self.state.standoff_controller and self.state.standoff_uav_idx is not None:
                    ctrl = self.state.standoff_controller
                    # Restore mission index and scan progress so we resume exactly where paused.
                    if hasattr(self.state, "standoff_resume_index"):
                        ctrl.resume_from(
                            self.state.standoff_resume_index,
                            getattr(self.state, "standoff_resume_scan_t", 0.0),
                        )
                    ctrl.heading_override_deg = None
                    leg_idx = max(0, min(ctrl.mission_index, len(ctrl.mission_plan) - 1))
                    curr_wp = ctrl.mission_plan[leg_idx].waypoint
                    next_wp = ctrl.mission_plan[leg_idx + 1].waypoint if leg_idx + 1 < len(ctrl.mission_plan) else None
                    if next_wp:
                        dx_leg = next_wp[0] - curr_wp[0]
                        dy_leg = next_wp[1] - curr_wp[1]
                        if abs(dx_leg) + abs(dy_leg) > 1e-3:
                            ctrl.entry_heading_deg = wrap_deg(math.degrees(math.atan2(-dy_leg, dx_leg)))
                # Let standoff controller regain gimbal on next tick
                self.state.standoff_gimbal_target = None
        # Failsafe: if tracking is off but standoff is still paused, unpause.
        if not self.state.tracking_active and getattr(self.state, "standoff_paused", False):
            self.state.standoff_paused = False
            self.state.standoff_gimbal_target = None
        # Update LAH QRF (attack/RTB)
        self._update_lah_qrf(dt_sim)
        update_autopilot(self.state, self.dem, dt_sim)
        self._update_line_scans(dt_sim)
        if self._debug_auto_steps > 0:
            self._debug_auto_steps -= 1
            for idx, mission in enumerate(self.state.missions):
                if mission and mission.waypoints and mission.active and not mission.completed:
                    wp = mission.waypoints[mission.current_idx]
                    u = self.state.uavs[idx]
                    dist = math.hypot(wp.x - u.s.x, wp.y - u.s.y)
                    print(
                        f"[auto-debug] craft{idx+1} wp {mission.current_idx+1}/{len(mission.waypoints)} "
                        f"dist2D={dist:.1f}m pos=({u.s.x:.1f},{u.s.y:.1f},{u.s.z:.1f}) "
                        f"wp=({wp.x:.1f},{wp.y:.1f},{wp.z:.1f})"
                    )

    def _update_targets_and_missiles(self, dt_sim):
        if self.state.targets_move:
            for t in self.state.targets:
                t.step(dt_sim, dem=self.dem)
        for m in list(self.state.missiles):
            m.step(dt_sim)
            if (not m.active) and (m.exploded and m.explode_time > 1.2 and len(m.sparks) == 0):
                self.state.missiles.remove(m)
        if self.debug_frames < 1:
            _log("after target/missile step")

    def _update_line_scans(self, dt_sim: float):
        if not getattr(self.state, "autopilot_enabled", False):
            return
        for idx, mission in enumerate(self.state.missions):
            if mission is None or mission.completed or not mission.active:
                self.state.line_scan_states[idx] = None
                self.state.gimbal_targets[idx] = None
                continue
            if mission.current_idx >= len(mission.waypoints):
                self.state.line_scan_states[idx] = None
                self.state.gimbal_targets[idx] = None
                continue
            wp = mission.waypoints[mission.current_idx]
            if wp.line_search and len(wp.line_search) >= 2:
                speed = wp.search_speed if wp.search_speed is not None else 0.0
                state = self.state.line_scan_states[idx]
                if state is None or getattr(state, "path", None) is not wp.line_search:
                    state = LineScanState(wp.line_search, speed)
                    self.state.line_scan_states[idx] = state
                    print(
                        f"[line-scan] craft{idx+1} wp{mission.current_idx+1}/{len(mission.waypoints)} "
                        f"segments={len(wp.line_search)} speed={speed:.1f} m/s"
                    )
                target = state.step(dt_sim)
                self.state.gimbal_targets[idx] = target
            else:
                self.state.line_scan_states[idx] = None
                self.state.gimbal_targets[idx] = None

    def _evaluate_threats(self, dt_sim):
        detection_lines = []
        cutoff2 = self.render_cutoff_m * self.render_cutoff_m
        if self.state.threat_kill_disabled:
            # Clear any lingering crippled flags while kills are disabled.
            for i in range(len(self.state.crippled)):
                self.state.crippled[i] = False
        for idx, u in enumerate(self.state.uavs):
            with self.state.uav_locks[idx]:
                upos = np.array([u.s.x, u.s.y, u.s.z], dtype=float)
            for t in self.state.targets:
                if not getattr(t, "threat", None):
                    continue
                dist2 = (t.x - upos[0]) ** 2 + (t.y - upos[1]) ** 2 + (t.z - upos[2]) ** 2
                if dist2 > cutoff2:
                    continue
                # If threat lethality is disabled, skip kill logic entirely.
                if self.state.threat_kill_disabled or getattr(getattr(t.threat, "weapon", None), "omega", 1.0) == 0.0:
                    # Still track detection for HUD lines but never apply damage.
                    los = check_los(upos, np.array([t.x, t.y, t.z]), self.dem)
                    t.threat.detection_prob(float(np.linalg.norm(np.array([t.x, t.y, t.z]) - upos)), los, dt_sim)
                    detection_lines.append(
                        {
                            "tpos": (t.x, t.y, t.z),
                            "upos": tuple(upos),
                            "los": los,
                            "detected": t.threat.state.detected,
                        }
                    )
                    continue
                range_m = float(np.linalg.norm(np.array([t.x, t.y, t.z]) - upos))
                if range_m < 1e-3:
                    continue
                los = check_los(upos, np.array([t.x, t.y, t.z]), self.dem)
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
                    with self.state.uav_locks[idx]:
                        self.state.crippled[idx] = True
                        u.cmd_throttle = -1.0
                        u.cmd_pitch_rate = -u.p.max_pitch_rate_dps * 0.8
                        spin_sign = 1.0 if random.random() < 0.5 else -1.0
                        u.cmd_roll_rate = u.p.max_roll_rate_dps * 0.6 * spin_sign
                        u.cmd_yaw_rate = u.p.max_yaw_rate_dps * 0.6 * spin_sign
        if self.debug_frames < 1:
            _log("after threat eval")
        return detection_lines

    def _remove_destroyed_targets(self):
        self.state.targets = [t for t in self.state.targets if getattr(t, "alive", True)]

    def _headless_detection_and_tracking(self, snapshots, active_snap, nearest):
        if not self.state or self.dem is None:
            return
        is_lah = self.state.uav_types[self.state.active_idx] == "LAH"
        standoff_target = getattr(self.state, "standoff_gimbal_target", None)
        standoff_uav_idx = getattr(self.state, "standoff_uav_idx", None)
        line_scan_target = None
        if 0 <= self.state.active_idx < len(self.state.gimbal_targets):
            line_scan_target = self.state.gimbal_targets[self.state.active_idx]
        active_pos = np.array([active_snap[0], active_snap[1], active_snap[2]], dtype=float)
        cutoff2 = self.render_cutoff_m * self.render_cutoff_m
        footprint_corners = None

        if standoff_target is not None and standoff_uav_idx is not None:
            cam_idx = standoff_uav_idx
            with self.state.uav_locks[cam_idx]:
                _, footprint_corners = draw_camera_footprint(
                    self.state.uavs[cam_idx],
                    standoff_target,
                    fov_diag_deg=self.state.fov_diag,
                    dem=self.dem,
                    draw=False,
                )
        elif line_scan_target is not None:
            tgt_pos = np.array(line_scan_target, dtype=float)
            with self.state.uav_locks[self.state.active_idx]:
                _, footprint_corners = draw_camera_footprint(
                    self.state.uavs[self.state.active_idx],
                    (float(tgt_pos[0]), float(tgt_pos[1]), float(tgt_pos[2])),
                    fov_diag_deg=self.state.fov_diag,
                    dem=self.dem,
                    draw=False,
                )
        elif nearest and (not is_lah):
            with self.state.uav_locks[self.state.active_idx]:
                _, footprint_corners = draw_camera_footprint(
                    self.state.uavs[self.state.active_idx],
                    (float(nearest.x), float(nearest.y), float(nearest.z)),
                    fov_diag_deg=self.state.fov_diag,
                    dem=self.dem,
                    draw=False,
                )

        if (
            not self.state.tracking_active
            and not getattr(self.state, "standoff_paused", False)
            and getattr(self.state, "tracking_cooldown", 0.0) <= 0.0
            and footprint_corners
            and self.auto_mode != "flightpath"
        ):
            tracking_uav_idx = standoff_uav_idx if standoff_uav_idx is not None else self.state.active_idx
            for tidx, t in enumerate(self.state.targets):
                if getattr(t, "alive", True):
                    dist2 = (t.x - active_pos[0]) ** 2 + (t.y - active_pos[1]) ** 2 + (t.z - active_pos[2]) ** 2
                    if dist2 > cutoff2:
                        continue
                    if self._point_in_poly(t.x, t.y, footprint_corners):
                        self._log_0402_detection(tracking_uav_idx + 1, tidx + 1, t)
                        self._start_tracking(tidx, tracking_uav_idx)
                        break

    def _render_frame(self, snapshots, active_snap, nearest, detection_lines, dt_sim, time_scale_val, uav_lon, uav_lat):
        is_lah = self.state.uav_types[self.state.active_idx] == "LAH"

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        self.cam.apply()

        standoff_ctrl = getattr(self.state, "standoff_controller", None)
        standoff_target = getattr(self.state, "standoff_gimbal_target", None)
        standoff_uav_idx = getattr(self.state, "standoff_uav_idx", None)
        line_scan_target = None
        if self.state and 0 <= self.state.active_idx < len(self.state.gimbal_targets):
            line_scan_target = self.state.gimbal_targets[self.state.active_idx]
        footprint_corners = None

        if self.state.fog_enabled:
            glEnable(GL_FOG)
        else:
            glDisable(GL_FOG)

        glDisable(GL_BLEND)
        draw_grid(size=int(RENDER_RADIUS_M), step=100)
        draw_axes(50)
        draw_dem(
            self.dem,
            self.cam,
            center_xy=(active_snap[0], active_snap[1]),
            radius_m=RENDER_RADIUS_M,
            prefetch_radius_m=DEM_PREFETCH_RADIUS_M,
            move_threshold=DEM_CACHE_MOVE_THRESHOLD_M,
            z_scale=1.0,
        )
        glEnable(GL_BLEND)

        for idx, trail in enumerate(self.state.trails):
            if len(trail) >= 2:
                glDisable(GL_LINE_STIPPLE)
                glLineWidth(2.0 if idx == self.state.active_idx else 1.5)
                if idx == self.state.active_idx:
                    glColor3f(0.2, 0.6, 1.0)
                else:
                    glColor3f(0.4, 0.5, 0.7)
                glBegin(GL_LINE_STRIP)
                for p in trail:
                    glVertex3f(*p)
                glEnd()

        for i, u in enumerate(self.state.uavs):
            with self.state.uav_locks[i]:
                if self.state.uav_types[i] == "LAH":
                    draw_lah(u, self.rotor_angle)
                else:
                    draw_uav(u)
        cutoff2 = self.render_cutoff_m * self.render_cutoff_m
        active_pos = np.array([active_snap[0], active_snap[1], active_snap[2]], dtype=float)
        for t in self.state.targets:
            dist2 = (t.x - active_pos[0]) ** 2 + (t.y - active_pos[1]) ** 2 + (t.z - active_pos[2]) ** 2
            if dist2 <= cutoff2:
                t.draw()
        for m in self.state.missiles:
            m.draw(self.dem)
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

        mission = self._mission_for_idx(self.state.active_idx)
        if mission and mission.waypoints and self.state.autopilot_enabled:
            glPointSize(7.0)
            glColor3f(0.95, 0.85, 0.2)
            glBegin(GL_POINTS)
            for wp in mission.waypoints:
                glVertex3f(wp.x, wp.y, wp.z)
            glEnd()
            glPointSize(1.0)
        if standoff_ctrl and standoff_ctrl.mission_plan and self.state.autopilot_enabled:
            glPointSize(7.0)
            glColor3f(0.95, 0.85, 0.2)
            glBegin(GL_POINTS)
            for pkg in standoff_ctrl.mission_plan:
                x, y, z = pkg.waypoint
                glVertex3f(x, y, z)
            glEnd()
            glPointSize(1.0)
            # Draw rejoin helper points if any.
            if getattr(standoff_ctrl, "rejoin_points_debug", None):
                draw_rejoin_points(standoff_ctrl.rejoin_points_debug)

        model = glGetDoublev(GL_MODELVIEW_MATRIX)
        proj = glGetDoublev(GL_PROJECTION_MATRIX)
        viewport = glGetIntegerv(GL_VIEWPORT)
        label_entries = []
        for idx, snap in enumerate(snapshots):
            sx, sy, sz = gluProject(snap[0], snap[1], snap[2] + 10.0, model, proj, viewport)
            if 0.0 <= sz <= 1.0:
                color = (255, 80, 80) if idx == self.state.active_idx else (220, 220, 220)
                name = f"LAH{idx + 1}" if idx < 3 else f"UAV{idx - 2}"
                speed_text = f"{snap[6]:.0f}m/s"
                state_tag = "FALL" if self.state.crippled[idx] else ("CRASH" if self.state.crashed[idx] else "")
                mission = self._mission_for_idx(idx)
                wp_info = ""
                if mission and mission.waypoints:
                    wp_info = f" crt_wp:{mission.current_idx + 1}/{len(mission.waypoints)}"
                text = f"{name} {speed_text}{wp_info}" if not state_tag else f"{name} {speed_text} {state_tag}{wp_info}"
                label_entries.append((sx, sy + 8, text, color))
        for tidx, t in enumerate(self.state.targets):
            dist2 = (t.x - active_pos[0]) ** 2 + (t.y - active_pos[1]) ** 2 + (t.z - active_pos[2]) ** 2
            if dist2 > cutoff2:
                continue
            sx, sy, sz = gluProject(t.x, t.y, t.z + 10.0, model, proj, viewport)
            if 0.0 <= sz <= 1.0:
                label_entries.append((sx, sy + 8, f"target{tidx + 1}", (255, 200, 80)))
        if mission and mission.waypoints and self.state.autopilot_enabled:
            for wp in mission.waypoints:
                sx, sy, sz = gluProject(wp.x, wp.y, wp.z + 2.0, model, proj, viewport)
                if 0.0 <= sz <= 1.0:
                    label_entries.append((sx, sy + 6, wp.name, (240, 210, 90)))
        draw_labels(label_entries)

        footprint_area = None
        los_clear = False
        if standoff_target is not None and standoff_uav_idx is not None:
            cam_idx = standoff_uav_idx
            uav_pos = np.array(
                [self.state.uavs[cam_idx].s.x, self.state.uavs[cam_idx].s.y, self.state.uavs[cam_idx].s.z],
                dtype=float,
            )
            los_clear = check_los(uav_pos, np.array(standoff_target, dtype=float), self.dem)
            with self.state.uav_locks[cam_idx]:
                footprint_area, footprint_corners = draw_camera_footprint(
                    self.state.uavs[cam_idx],
                    standoff_target,
                    fov_diag_deg=self.state.fov_diag,
                    dem=self.dem,
                    aspect_ratio=16 / 9,
                    z_scale=1.0,
                )
        elif line_scan_target is not None:
            uav_pos = np.array([active_snap[0], active_snap[1], active_snap[2]], dtype=float)
            tgt_pos = np.array(line_scan_target, dtype=float)
            los_clear = check_los(uav_pos, tgt_pos, self.dem)
            glLineWidth(2.0)
            glEnable(GL_LINE_STIPPLE)
            glLineStipple(1, 0x3333)
            glColor3f(0.6, 0.8, 0.6)
            glBegin(GL_LINES)
            glVertex3f(*uav_pos)
            glVertex3f(*tgt_pos)
            glEnd()
            glDisable(GL_LINE_STIPPLE)
            with self.state.uav_locks[self.state.active_idx]:
                footprint_area, footprint_corners = draw_camera_footprint(
                    self.state.uavs[self.state.active_idx],
                    (tgt_pos[0], tgt_pos[1], tgt_pos[2]),
                    fov_diag_deg=self.state.fov_diag,
                    dem=self.dem,
                    aspect_ratio=16 / 9,
                    z_scale=1.0,
                )
        elif nearest and (not is_lah):
            uav_pos = np.array([active_snap[0], active_snap[1], active_snap[2]], dtype=float)
            tgt_pos = np.array([nearest.x, nearest.y, nearest.z], dtype=float)
            los_clear = check_los(uav_pos, tgt_pos, self.dem)
            glLineWidth(2.0)
            glEnable(GL_LINE_STIPPLE)
            glLineStipple(1, 0x3333)
            glColor3f(0.6, 0.6, 0.6)
            glBegin(GL_LINES)
            glVertex3f(*uav_pos)
            glVertex3f(*tgt_pos)
            glEnd()
            glDisable(GL_LINE_STIPPLE)
            with self.state.uav_locks[self.state.active_idx]:
                footprint_area, footprint_corners = draw_camera_footprint(
                    self.state.uavs[self.state.active_idx],
                    (nearest.x, nearest.y, nearest.z),
                    fov_diag_deg=self.state.fov_diag,
                    dem=self.dem,
                    aspect_ratio=16 / 9,
                    z_scale=1.0,
                )

        # Trigger tracking if ANY target enters the camera footprint (respect cooldown and pause state)
        if (
            not self.state.tracking_active
            and not getattr(self.state, "standoff_paused", False)
            and getattr(self.state, "tracking_cooldown", 0.0) <= 0.0
            and footprint_corners
            and self.auto_mode != "flightpath"
        ):
            # Prefer standoff UAV if available, otherwise use currently active craft.
            tracking_uav_idx = standoff_uav_idx if standoff_uav_idx is not None else self.state.active_idx
            for tidx, t in enumerate(self.state.targets):
                if getattr(t, "alive", True):
                    dist2 = (t.x - active_pos[0]) ** 2 + (t.y - active_pos[1]) ** 2 + (t.z - active_pos[2]) ** 2
                    if dist2 > cutoff2:
                        continue
                    if self._point_in_poly(t.x, t.y, footprint_corners):
                        self._log_0402_detection(tracking_uav_idx + 1, tidx + 1, t)
                        self._start_tracking(tidx, tracking_uav_idx)
                        break

        lines = [
            f"Active: {'LAH' if is_lah else 'UAV'}{self.state.active_idx + 1}  pos (m): x={active_snap[0]:7.1f}  y={active_snap[1]:7.1f}  z={active_snap[2]:6.1f}",
            f"pos (lat/lon): lat={uav_lat:9.5f}  lon={uav_lon:10.5f}",
            f"spd (m/s): {active_snap[6]:5.1f}   yaw={active_snap[5]:6.1f}deg  pitch={active_snap[4]:5.1f}deg  roll={active_snap[3]:5.1f}deg",
            f"FOV (diag): {self.state.fov_diag:4.1f}deg   LOS: {1 if los_clear else 0}   dt={dt_sim*1000:.1f}ms  time x{time_scale_val:.1f}",
            f"Targets: {len(self.state.targets)}  Missiles: {len(self.state.missiles)}",
            "1-6: switch craft (1-3 LAH, 4-6 UAV) | M: Fire (LAH only) | N: Toggle targets | G: Spawn target | P: Autopilot WPs | Wheel: Zoom | RMB drag: Orbit | MMB drag: Pan",
        ]
        if mission and mission.waypoints:
            if self._autopilot_active_for(self.state.active_idx):
                status = "ON"
            elif mission.completed:
                status = "DONE"
            elif self.state.autopilot_enabled:
                status = "HOLD"
            else:
                status = "OFF"
            lines.append(f"Autopilot: {status} wp {mission.current_idx + 1}/{len(mission.waypoints)} (P to regenerate)")
        if standoff_ctrl:
            total = len(standoff_ctrl.mission_plan)
            current_leg = min(standoff_ctrl.mission_index + 1, total) if total else 0
            status = "DONE" if standoff_ctrl.mission_done else ("ON" if self.state.autopilot_enabled else "OFF")
            lines.append(f"Standoff: {status} leg {current_leg}/{total} (P to restart)")
        if footprint_area is not None:
            lines.append(f"Footprint area: {footprint_area:8.1f} m^2")
        if any(self.state.crippled) and not any(self.state.crashed):
            crippled_ids = [str(i + 1) for i, c in enumerate(self.state.crippled) if c]
            lines.append(f"Status: DAMAGED UAVs: {', '.join(crippled_ids)} (falling)")
        if any(self.state.crashed):
            crashed_ids = [str(i + 1) for i, c in enumerate(self.state.crashed) if c]
            lines.append(f"Status: CRASHED craft: {', '.join(crashed_ids)} (press R to reset)")
        draw_hud(lines)

        if self.debug_frames < 3:
            print(
                f"[frame] cam_target=({self.cam.target[0]:.1f},{self.cam.target[1]:.1f},{self.cam.target[2]:.1f}) "
                f"cam_dist={self.cam.distance:.1f} uav_pos=({active_snap[0]:.1f},{active_snap[1]:.1f},{active_snap[2]:.1f}) "
                f"time_scale={time_scale_val:.1f} fog={self.state.fog_enabled}"
            )
            err = glGetError()
            if err:
                print(f"[GL ERROR] frame: {err}")
            self.debug_frames += 1

        if self.show_minimap:
            self._draw_minimap(snapshots)

        pygame.display.flip()
        if self.debug_frames < 2:
            _log("after display flip")

    def _shutdown(self):
        if self.state:
            self.state.run_event.clear()
            for t in self.state.workers:
                t.join(timeout=0.2)
        if self.log_stop_event:
            self.log_stop_event.set()
        if self.log_thread:
            self.log_thread.join(timeout=0.5)
        pygame.quit()
